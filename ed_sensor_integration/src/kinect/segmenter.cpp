#include "ed/kinect/segmenter.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <rgbd/image.h>
#include <rgbd/view.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <ros/console.h>

// Clustering
#include <queue>
#include <ed/convex_hull_calc.h>
#include "ros_segment_inference.h"
#include <sys/resource.h>
// Visualization
//#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------
void printMemoryUsage(const std::string& label) {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    ROS_INFO("%s - Memory usage: %ld KB", label.c_str(), usage.ru_maxrss);
}

Segmenter::Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

Segmenter::~Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

namespace
{

class DepthRenderer : public geo::RenderResult
{

public:

    DepthRenderer(cv::Mat& z_buffer_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_)
    {
    }

    void renderPixel(int x, int y, float depth, int /*i_triangle*/)
    {
        float& old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            old_depth = depth;
        }
    }

    cv::Mat& z_buffer;
};

}

// ----------------------------------------------------------------------------------------------------

void Segmenter::removeBackground(cv::Mat& depth_image, const ed::WorldModel& world, const geo::DepthCamera& cam,
                                 const geo::Pose3D& sensor_pose, double background_padding)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render the world model as seen by the depth sensor

    cv::Mat depth_model(depth_image.rows, depth_image.cols, CV_32FC1, 0.0);

    DepthRenderer res(depth_model);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (!e->visual() || !e->has_pose())
            continue;

        geo::RenderOptions opt;
        opt.setMesh(e->visual()->getMesh(), sensor_pose.inverse() * e->pose());
        cam.render(opt, res);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Filter all points that can be associated with the rendered depth image

    unsigned int size = depth_image.rows * depth_image.cols;
    for(unsigned int i = 0; i < size; ++i)
    {
        float& ds = depth_image.at<float>(i);
        float dm = depth_model.at<float>(i);
        if (dm > 0 && ds > 0 && ds > dm - background_padding)
            ds = 0;
    }
}

// ----------------------------------------------------------------------------------------------------

class MinMaxRenderer : public geo::RenderResult
{

public:

    MinMaxRenderer(int width, int height) : geo::RenderResult(width, height)
    {
        min_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
        max_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
    }

    void renderPixel(int x, int y, float depth, int /*i_triangle*/)
    {
        // TODO: now the renderer can only deal with convex meshes, which means
        // that at each pixel there can only be one minimum and one maximum pixel
        // There is an easy solution for concave meshes: determine which side
        // the triangle points (away or to the camera) and test the pixel in the depth
        // image to be on the correct side. ... etc ...

        float& d_min = min_buffer.at<float>(y, x);
        float& d_max = max_buffer.at<float>(y, x);

        if (d_min == 0 || depth < d_min)
            d_min = depth;

        d_max = std::max(d_max, depth);
    }

    cv::Mat min_buffer;
    cv::Mat max_buffer;

};

// ----------------------------------------------------------------------------------------------------

void Segmenter::calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                                      const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image) const
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render shape

    const cv::Mat& depth_image = image.getDepthImage();

    rgbd::View view(image, depth_image.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    MinMaxRenderer res(depth_image.cols, depth_image.rows);

    geo::RenderOptions opt;
    opt.setBackFaceCulling(false);
    opt.setMesh(shape.getMesh(), shape_pose);

    cam_model.render(opt, res);

    filtered_depth_image = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1, 0.0);

    for(int i = 0; i < depth_image.cols * depth_image.rows; ++i)
    {
        float d = depth_image.at<float>(i);
        if (d <= 0)
            continue;

        float d_min = res.min_buffer.at<float>(i);
        float d_max = res.max_buffer.at<float>(i);

        if (d_min > 0 && d_max > 0 && d >= d_min && d <= d_max)
            filtered_depth_image.at<float>(i) = d;
    }

//    cv::imshow("min", res.min_buffer / 10);
//    cv::imshow("max", res.max_buffer / 10);
//    cv::imshow("diff", (res.max_buffer - res.min_buffer) * 10);
//    cv::imshow("filtered", filtered_depth_image / 10);
//    cv::waitKey();
}

// ----------------------------------------------------------------------------------------------------

cv::Mat Segmenter::preprocessRGBForSegmentation(const cv::Mat& rgb_image, const cv::Mat& filtered_depth_image) const {
    // Apply depth mask to RGB
    cv::Mat masked_rgb = cv::Mat::zeros(rgb_image.size(), rgb_image.type());
    for (int y = 0; y < rgb_image.rows; y++) {
        for (int x = 0; x < rgb_image.cols; x++) {
            int idx = y * rgb_image.cols + x;
            if (filtered_depth_image.at<float>(idx) > 0) {
                masked_rgb.at<cv::Vec3b>(y, x) = rgb_image.at<cv::Vec3b>(y, x);
            }
        }
    }
    return masked_rgb;
}
// ----------------------------------------------------------------------------------------------------

std::vector<cv::Mat> Segmenter::cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                        const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters, const cv::Mat& rgb_image) const
{
    int width = depth_image.cols;
    int height = depth_image.rows;
    ROS_DEBUG("Cluster with depth image of size %i, %i", width, height);
    //printMemoryUsage("Before inference");
    std::vector<cv::Mat> masks = DetectTest(rgb_image.clone());
    //printMemoryUsage("After inference");
    ROS_DEBUG("Creating clusters");
    unsigned int size = width * height;

    for (size_t i = 0; i < masks.size(); ++i)
    {
        cv::Mat mask = masks[i];
        clusters.push_back(EntityUpdate());
        EntityUpdate& cluster = clusters.back();

        unsigned int num_points = 0;

         // Add to cluster
        for(int y = 0; y < mask.rows; ++y)
        {
            for(int x = 0; x < mask.cols; ++x)
            {
                if (mask.at<unsigned char>(y, x) > 0) {
                    unsigned int pixel_idx = y * width + x;
                    float d = depth_image.at<float>(pixel_idx);

                    if (d > 0 && std::isfinite(d)) {
                        // NEW: Check neighboring depths for consistency
                        bool consistent_depth = true;
                        float sum_valid_neighbors = 0;
                        float sum_depths = 0;
                        int num_valid = 0;

                        // Check 8-connected neighborhood
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                if (dx == 0 && dy == 0) continue;

                                int nx = x + dx;
                                int ny = y + dy;

                                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                    float nd = depth_image.at<float>(ny * width + nx);
                                    if (nd > 0 && std::isfinite(nd)) {
                                        sum_depths += nd;
                                        num_valid++;
                                    }
                                }
                            }
                        }

                        // Only include point if depth is consistent with neighbors
                        if (num_valid > 3) {  // At least 4 valid neighbors
                            float avg_depth = sum_depths / num_valid;
                            if (std::abs(d - avg_depth) < 0.05) {  // Within 5cm
                                cluster.pixel_indices.push_back(pixel_idx);
                                cluster.points.push_back(cam_model.project2Dto3D(x, y) * d);
                            }
                        }
                        else {
                            std::cerr << "WARNINGGGGGGGGGGG: Not enough valid neighbors for pixel (" << x << ", " << y << ") with depth " << d << std::endl;
                        }
                    }
                }
            }
        }

        // Check if cluster has enough points. If not, remove it from the list
        if (cluster.pixel_indices.size() < 100) // TODO: magic number
        {
            clusters.pop_back();
            continue;
        }

        // Calculate cluster convex hull
        float z_min = 1e9;
        float z_max = -1e9;

        // Calculate z_min and z_max of cluster
        ROS_DEBUG("Computing min and max of cluster");
        std::vector<geo::Vec2f> points_2d(cluster.points.size());
        for(unsigned int j = 0; j < cluster.points.size(); ++j)
        {
            const geo::Vec3& p = cluster.points[j];

            // Transform sensor point to map frame
            geo::Vector3 p_map = sensor_pose * p;

            points_2d[j] = geo::Vec2f(p_map.x, p_map.y);

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);
        }

        ed::convex_hull::create(points_2d, z_min, z_max, cluster.chull, cluster.pose_map);
        cluster.chull.complete = false;
    }
    return masks;
}

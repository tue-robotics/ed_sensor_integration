#include "ed/kinect/segmenter.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

// Clustering
#include <queue>
#include <ed/convex_hull_calc.h>

// Visualization
//#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

class MinMaxRenderer : public geo::RenderResult
{

public:

    MinMaxRenderer(int width, int height) : geo::RenderResult(width, height)
    {
        min_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
        max_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        // TODO: now the renderer can only deal with convex meshes, which means
        // that at each pixel there can only be one minimum and one maximum pixel
        // There is an easy solution for concave meshes: determine which side
        // the triangle points (away or to the camera) and test the pixel in the depth
        // image to be on the correct side. ... etc ...

        float d_min = min_buffer.at<float>(y, x);
        if (d_min == 0)
        {
            min_buffer.at<float>(y, x) = depth;
        }
        else if (depth < d_min)
        {
            min_buffer.at<float>(y, x) = depth;
            max_buffer.at<float>(y, x) = d_min;
        }
        else
        {
            max_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat min_buffer;
    cv::Mat max_buffer;

};

// ----------------------------------------------------------------------------------------------------

Segmenter::Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

Segmenter::~Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

void Segmenter::calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                                      const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image)
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

        if (d_min > 0 && d_max > 0 && d > d_min && d < d_max)
            filtered_depth_image.at<float>(i) = d;
    }

//    cv::imshow("min", res.min_buffer / 10);
//    cv::imshow("max", res.max_buffer / 10);
//    cv::imshow("filtered", filtered_depth_image / 10);
//    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

void Segmenter::cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                        const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters)
{
    int width = depth_image.cols;
    int height = depth_image.rows;

    cv::Mat visited(height, width, CV_8UC1, cv::Scalar(0));

    // Mark borders as visited (2-pixel border)
    for(int x = 0; x < width; ++x)
    {
        visited.at<unsigned char>(0, x) = 1;
        visited.at<unsigned char>(1, x) = 1;
        visited.at<unsigned char>(height - 1, x) = 1;
        visited.at<unsigned char>(height - 2, x) = 1;
    }

    for(int y = 0; y < height; ++y)
    {
        visited.at<unsigned char>(y, 0) = 1;
        visited.at<unsigned char>(y, 1) = 1;
        visited.at<unsigned char>(y, width - 1) = 1;
        visited.at<unsigned char>(y, width - 2) = 1;
    }

    int dirs[] = { -1, 1, -width, width,
                   -2, 2, -width * 2, width * 2};  // Also try one pixel skipped (filtering may cause some 1-pixel gaps)

    for(unsigned int i_pixel = 0; i_pixel < width * height; ++i_pixel)
    {
        float d = depth_image.at<float>(i_pixel);

        if (d == 0)
            continue;

        // Create cluster
        clusters.push_back(EntityUpdate());
        EntityUpdate& cluster = clusters.back();

        // Mark visited
        visited.at<unsigned char>(i_pixel) = 1;

        std::queue<unsigned int> Q;
        Q.push(i_pixel);

        while(!Q.empty())
        {
            unsigned int p1 = Q.front();
            Q.pop();

            float p1_d = depth_image.at<float>(p1);

            // Add to cluster
            cluster.pixel_indices.push_back(p1);
            cluster.points.push_back(cam_model.project2Dto3D(p1 % width, p1 / width) * p1_d);

            for(int dir = 0; dir < 8; ++dir)
            {
                unsigned int p2 = p1 + dirs[dir];
                float p2_d = depth_image.at<float>(p2);

                // If not yet visited, and depth is within bounds
                if (visited.at<unsigned char>(p2) == 0 && std::abs<float>(p2_d - p1_d) < 0.05)
                {
                    // Mark visited
                    visited.at<unsigned char>(p2) = 1;

                    // Add point to queue
                    Q.push(p2);
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
}

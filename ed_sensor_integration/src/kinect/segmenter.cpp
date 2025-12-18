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
#include <sys/resource.h>

#include <opencv2/ml.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

#include <ed_sensor_integration/kinect/segmodules/sam_seg_module.h>
#include <bmm/bayesian_mixture_model.hpp>

// ----------------------------------------------------------------------------------------------------

Segmenter::Segmenter(tue::Configuration config)
    : config_(config)
{
}

// ----------------------------------------------------------------------------------------------------

Segmenter::~Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

namespace
{
// Internal constants (tuning thresholds)
constexpr std::size_t MIN_FILTERED_POINTS = 10;
constexpr double      MIN_RETENTION_RATIO = 0.10;  // 10%
constexpr std::size_t MIN_CLUSTER_POINTS = 100;

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
}

// ----------------------------------------------------------------------------------------------------

cv::Mat Segmenter::preprocessRGBForSegmentation(const cv::Mat& rgb_image,
                                                const cv::Mat& filtered_depth_image) const
{
    cv::Mat masked_rgb = cv::Mat::zeros(rgb_image.size(), rgb_image.type());
    for (int y = 0; y < rgb_image.rows; ++y) {
        const float* depth_row = filtered_depth_image.ptr<float>(y); // fast
        cv::Vec3b* out_row = masked_rgb.ptr<cv::Vec3b>(y);
        const cv::Vec3b* rgb_row = rgb_image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < rgb_image.cols; ++x) {
            if (depth_row[x] > 0.f) {
                out_row[x] = rgb_row[x];
            }
        }
    }
    return masked_rgb;
}
// ----------------------------------------------------------------------------------------------------

std::vector<cv::Mat> Segmenter::cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                        const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters, const cv::Mat& rgb_image, bool logging)
{
    int width = depth_image.cols;
    int height = depth_image.rows;
    ROS_DEBUG("Cluster with depth image of size %i, %i", width, height);

    std::vector<cv::Mat> masks = SegmentationPipeline(rgb_image.clone(), config_);
    ROS_DEBUG("Creating clusters");

    // Pre-allocate temporary storage (one per mask, avoid push_back races)
    std::vector<EntityUpdate> temp_clusters(masks.size());
    std::vector<bool> valid_cluster(masks.size(), false);

    // Parallel loop - each iteration is independent
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < masks.size(); ++i)
    {
        const cv::Mat& mask = masks[i];
        EntityUpdate cluster;  // local to this thread

        // Extract points from mask
        for(int y = 0; y < mask.rows; ++y) {
            for(int x = 0; x < mask.cols; ++x) {
                if (mask.at<unsigned char>(y, x) > 0) {
                    unsigned int pixel_idx = y * width + x;
                    float d = depth_image.at<float>(pixel_idx);

                    if (d > 0 && std::isfinite(d)) {
                        cluster.pixel_indices.push_back(pixel_idx);
                        cluster.points.push_back(cam_model.project2Dto3D(x, y) * d);
                    }
                }
            }
        }

        // Skip small clusters (< 100 points)
        if (cluster.pixel_indices.size() < MIN_CLUSTER_POINTS) {
            continue;  // valid_cluster[i] remains false
        }

        // BMM point cloud denoising
        GMMParams params;
        config_.value("psi0", params.psi0);
        config_.value("nu0", params.nu0);
        config_.value("alpha", params.alpha);
        config_.value("kappa0", params.kappa0);


        MAPGMM gmm(2, cluster.points, params); // 2 components: object + outliers
        gmm.fit(cluster.points, sensor_pose);
        // Get component assignments and inlier component
        std::vector<int> labels = gmm.get_labels();
        int inlier_component = gmm.get_inlier_component();

        // Filter points
        std::vector<geo::Vec3> filtered_points;
        std::vector<geo::Vec3> outlier_points;  // Only populate if needed

        for (size_t j = 0; j < labels.size(); j++)
        {
            if (labels[j] == inlier_component)
            {
                filtered_points.push_back(cluster.points[j]);
            }
            else if (logging)
            {
                outlier_points.push_back(cluster.points[j]);
            }
        }

        // Safety check: only use filtered points if we retained enough
        if (filtered_points.size() > MIN_FILTERED_POINTS &&
            filtered_points.size() > MIN_RETENTION_RATIO * cluster.points.size()) {
            // Use filtered points
            cluster.points = filtered_points;
            if (logging)
            {
                cluster.outlier_points = outlier_points;
                // Transform outlier points to map frame
                // for (size_t j = 0; j < cluster.outlier_points.size(); ++j) {
                //     cluster.outlier_points[j] = sensor_pose * cluster.outlier_points[j];
                // }
            }
        }
        else
        {
            // Safety check failed - keep original unfiltered points
            // Don't populate outlier_points since we're not using the GMM result
            ROS_DEBUG("GMM filtering rejected: retained %zu/%zu points",
                      filtered_points.size(), cluster.points.size());
        }

        // Calculate convex hull
        float z_min = 1e9;
        float z_max = -1e9;
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

        // Store in thread-safe pre-allocated array
        temp_clusters[i] = cluster;
        valid_cluster[i] = true;
    }

    // Sequential section: collect valid clusters
    clusters.clear();
    clusters.reserve(masks.size());
    for (size_t i = 0; i < temp_clusters.size(); ++i) {
        if (valid_cluster[i]) {
            clusters.push_back(std::move(temp_clusters[i]));
        }
    }

    return masks;
}

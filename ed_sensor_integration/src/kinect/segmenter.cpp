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

#include <opencv2/ml.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>
#include "ed/kinect/bayesian_gmm.h"
#include "ed/kinect/variational_gmm.h"

void applyDBSCANFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose, tue::Configuration config_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Build point cloud in map frame
    for (const auto& point : cluster.points) {
        geo::Vec3 p_map = sensor_pose * point;
        cloud->push_back(pcl::PointXYZ(p_map.x, p_map.y, p_map.z));
    }

    // Create KdTree for search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Initialize clustering parameters from config
    double eps = 0.02;         // Default: 2cm
    int min_samples = 30;
    config_.value("eps", eps);
    config_.value("min_samples", min_samples);


    // Run clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // This is the DBSCAN "epsilon" (ε) parameter.
    // Defines the radius (2cm) in which to search for neighboring points
    // Too small: Objects fragment into multiple clusters | Too large: Different objects merge together
    ec.setClusterTolerance(eps);  // 2cm
    // Minimum number of points required to form a valid cluster
    // Smaller values: More sensitive to noise and small objects
    // Larger values: Eliminates smaller objects but reduces noise
    ec.setMinClusterSize(min_samples);
    // Maximum number of points allowed in a cluster. Should be large enough for your largest expected object
    ec.setMaxClusterSize(25000);
    // Specifies the spatial indexing structure for neighbor searches
    // KdTree is efficient for finding neighbors in 3D space
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Select largest cluster
    size_t largest_idx = 0;
    size_t largest_size = 0;

    for (size_t i = 0; i < cluster_indices.size(); i++) {
        if (cluster_indices[i].indices.size() > largest_size) {
            largest_size = cluster_indices[i].indices.size();
            largest_idx = i;
        }
    }

    // Filter points
    if (!cluster_indices.empty()) {
        std::vector<geo::Vec3> filtered_points;
        for (const auto& idx : cluster_indices[largest_idx].indices) {
            filtered_points.push_back(cluster.points[idx]);
        }
        cluster.points = filtered_points;
    }
}
// Visualization
//#include <opencv2/highgui/highgui.hpp>

// After collecting points in cluster.points:
void applyGMMFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose) {
    if (cluster.points.size() < 50) return;  // Too few points

    // Convert points to OpenCV Mat (N x 3)
    cv::Mat samples(cluster.points.size(), 3, CV_32F);
    for (size_t i = 0; i < cluster.points.size(); i++) {
        // Transform to map frame for consistent clustering
        geo::Vec3 p_map = sensor_pose * cluster.points[i];
        samples.at<float>(i, 0) = p_map.x;
        samples.at<float>(i, 1) = p_map.y;
        samples.at<float>(i, 2) = p_map.z;
    }

    // Create and train EM model
    cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
    em_model->setClustersNumber(2);  // Object + outliers
    em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);
    em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 0.001));

    // Train the model
    cv::Mat labels, probs;
    if (!em_model->trainEM(samples, cv::noArray(), labels, probs)) {
        ROS_WARN("GMM training failed, skipping filtering");
        return;
    }

    // Count points per component
    std::map<int, int> component_counts;
    for (int i = 0; i < labels.rows; i++) {
        component_counts[labels.at<int>(i, 0)]++;
    }

    // Find component with most points (argmax)
    int main_component = 0;
    int max_count = 0;
    for (const auto& pair : component_counts) {
        if (pair.second > max_count) {
            max_count = pair.second;
            main_component = pair.first;
        }
    }

    // Keep all points from the largest component
    std::vector<geo::Vec3> filtered_points;
    for (int i = 0; i < labels.rows; i++) {
        if (labels.at<int>(i, 0) == main_component) {
            filtered_points.push_back(cluster.points[i]);
        }
    }

    cluster.points = filtered_points;

    // Only replace if we kept some points
    //if (filtered_points.size() > cluster.points.size() * 0.1) {
        //cluster.points = filtered_points;
    //}
}

void applyVariationalBayesianGMMFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose) {
    if (cluster.points.size() < 50) return;

    // Create and fit VB-GMM
    VBGMM vbgmm(2, cluster.points); // 2 components: object + outliers
    vbgmm.fit(cluster.points, sensor_pose);

    // Get results
    std::vector<int> labels = vbgmm.get_labels();
    int inlier_component = vbgmm.get_inlier_component();
    double lower_bound = vbgmm.get_lower_bound();

    ROS_INFO("VB-GMM lower bound: %.3f", lower_bound);

    // Filter points based on component assignment
    std::vector<geo::Vec3> filtered_points;
    for (size_t i = 0; i < labels.size(); i++) {
        if (labels[i] == inlier_component) {
            filtered_points.push_back(cluster.points[i]);
        }
    }

    if (!filtered_points.empty()) {
        cluster.points = filtered_points;
        ROS_INFO("VB filtering: kept %zu of %zu points",
                filtered_points.size(), cluster.points.size());
    }
}

// ----------------------------------------------------------------------------------------------------
void printMemoryUsage(const std::string& label) {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    ROS_INFO("%s - Memory usage: %ld KB", label.c_str(), usage.ru_maxrss);
}

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
                        const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters, const cv::Mat& rgb_image)
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
        for(int y = 0; y < mask.rows; ++y) {
            for(int x = 0; x < mask.cols; ++x) {
                if (mask.at<unsigned char>(y, x) > 0) {
                    unsigned int pixel_idx = y * width + x;
                    float d = depth_image.at<float>(pixel_idx);

                    if (d > 0 && std::isfinite(d)) {
                        // NEW: Check neighboring depths for consistency with relative threshold
                        float sum_depths = 0;
                        int num_valid = 0;
                        std::vector<float> neighbor_depths;

                        // Check 8-connected neighborhood
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                if (dx == 0 && dy == 0) continue;

                                int nx = x + dx;
                                int ny = y + dy;

                                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                    // Only consider neighbors that are also in the mask
                                    if (mask.at<unsigned char>(ny, nx) > 0) {
                                        float nd = depth_image.at<float>(ny * width + nx);
                                        if (nd > 0 && std::isfinite(nd)) {
                                            sum_depths += nd;
                                            num_valid++;
                                            neighbor_depths.push_back(nd);
                                        }
                                    }
                                }
                            }
                        }

                        // Only include point if depth is consistent with neighbors
                        if (num_valid >= 3) {  // At least 3 valid neighbors
                            float avg_depth = sum_depths / num_valid;

                            // Use relative threshold - tighter for closer objects
                            float threshold = 0.03 * avg_depth;  // 3% of distance

                            // Check if the point is an outlier using median instead of mean
                            std::sort(neighbor_depths.begin(), neighbor_depths.end());
                            float median_depth = neighbor_depths[neighbor_depths.size()/2];

                            if (std::abs(d - median_depth) < threshold) {
                                // Add this point to the cluster
                                cluster.pixel_indices.push_back(pixel_idx);
                                cluster.points.push_back(cam_model.project2Dto3D(x, y) * d);
                            }
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


        // After collecting all points in the cluster but before convex hull calculation
        //applyDBSCANFiltering(cluster, sensor_pose, config_);
        //applyGMMFiltering(cluster, sensor_pose);
        //applyVariationalBayesianGMMFiltering(cluster, sensor_pose);

        ///////////////////// apply bayesian GMM filtering /////////////////////
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
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i] == inlier_component) {
                filtered_points.push_back(cluster.points[i]);
            }
        }
        // Safety check
        if (filtered_points.size() > 10 && filtered_points.size() > 0.1 * cluster.points.size()) {
            ROS_INFO("MAP-GMM filtering: kept %zu of %zu points (%.1f%%)",
                    filtered_points.size(), cluster.points.size(),
                    100.0 * filtered_points.size() / cluster.points.size());
            cluster.points = filtered_points;
        } else {
            ROS_WARN("MAP-GMM filtering: too few points kept, using original points");
        }

        ///////////////////// END: apply bayesian GMM filtering /////////////////////

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

        // After collecting all points, filter outliers
        if (!cluster.points.empty()) {
            // Calculate mean and standard deviation of distances to centroid
            geo::Vec3 centroid(0, 0, 0);
            for (const auto& p : cluster.points)
                centroid += p;
            centroid = centroid / cluster.points.size();

            // Calculate standard deviation
            float sum_sq_dist = 0;
            for (const auto& p : cluster.points)
                sum_sq_dist += (p - centroid).length2();
            float stddev = std::sqrt(sum_sq_dist / cluster.points.size());

            // Filter points that are too far from centroid
            std::vector<geo::Vec3> filtered_points;
            for (const auto& p : cluster.points) {
                if ((p - centroid).length() < 2.5 * stddev)
                    filtered_points.push_back(p);
            }

            cluster.points = filtered_points;
        }
    }
    return masks;
}

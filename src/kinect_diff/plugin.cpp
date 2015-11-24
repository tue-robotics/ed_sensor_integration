#include "plugin.h"

#include <geolib/datatypes.h>
#include <geolib/ros/tf_conversions.h>
#include <rgbd/Image.h>

#include <opencv2/highgui/highgui.hpp>

#include <ed/convex_hull.h>
#include <ed/convex_hull_calc.h>
#include <ed/update_request.h>
#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/mask.h>
#include <ed/measurement.h>

#include <rgbd/View.h>

// ----------------------------------------------------------------------------------------------------

struct Cluster
{
    // Measurement
    std::vector<unsigned int> pixel_indices;
    std::vector<geo::Vec3> points;

    // Shape
    ed::ConvexHull chull;

    // Pose
    geo::Pose3D pose_map;
};

// ----------------------------------------------------------------------------------------------------

void cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
             const geo::Pose3D& sensor_pose, int min_num_pixels, double depth_threshold,
             std::vector<Cluster>& clusters)
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
        clusters.push_back(Cluster());
        Cluster& cluster = clusters.back();

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
                if (visited.at<unsigned char>(p2) == 0 && std::abs<float>(p2_d - p1_d) < depth_threshold)
                {
                    // Mark visited
                    visited.at<unsigned char>(p2) = 1;

                    // Add point to queue
                    Q.push(p2);
                }
            }
        }

        // Check if cluster has enough points. If not, remove it from the list
        if (cluster.pixel_indices.size() < min_num_pixels)
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

        if (cluster.chull.height() < 0.5)
            clusters.pop_back();
    }
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin:: initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    if (config.value("topic", topic_))
    {
        std::cout << "[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic_ << "'." << std::endl;
        kinect_client_.intialize(topic_);
    }

    config.value("min_distance_sq", min_dist_sq_);
    config.value("clear_entities", clear_entities_);


    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr image = kinect_client_.nextImage();
    if (image)
        image_buffer_.push(image);

    if (image_buffer_.empty())
        return;

    image = image_buffer_.front();

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    geo::Pose3D sensor_pose;

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", image->getFrameId(), ros::Time(image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
        image_buffer_.pop();

    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform("map", image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(image->getTimestamp()) )
            {
                image_buffer_.pop();
                ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(image->getTimestamp()));
            }

            return;
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            return;
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Calculate background depth image

    const cv::Mat& depth = image->getDepthImage();
    int size = depth.cols * depth.rows;

    if (!background_depth_.data)
    {
        background_depth_ = image->getDepthImage().clone();
    }
    else
    {
        for(unsigned int i = 0; i < size; ++i)
        {
            float ds = depth.at<float>(i);
            if (ds == 0 || ds != ds)
                continue;

            float& db = background_depth_.at<float>(i);
            if (db == 0 || ds > db)
                db = ds;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Calculate diff

    cv::Mat diff(depth.rows, depth.cols, CV_32FC1, 0.0);
    for(unsigned int i = 0; i < size; ++i)
    {
        float ds = depth.at<float>(i);
        float db = background_depth_.at<float>(i);

        if (ds > 0 && ds == ds && db > 0 && ds < 0.9 * db)
            diff.at<float>(i) = ds;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Cluster

    int min_cluster_size_pixels = 1000;
    double depth_threshold = 0.6;

    rgbd::View view(*image, depth.cols);

    std::vector<Cluster> clusters;
    cluster(diff, view.getRasterizer(), sensor_pose, min_cluster_size_pixels, depth_threshold, clusters);

//    std::cout << "Num clusters: " << clusters.size() << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Associate

    std::set<ed::UUID> ids;
    std::vector<ed::UUID> associated_ids(clusters.size());

    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
        const Cluster& cluster = clusters[i];

        ed::UUID best_id;
        double min_dist_sq = min_dist_sq_;
        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape() || !e->has_pose() || e->convexHull().points.empty())
                continue;

            double dist_sq = (cluster.pose_map.t - e->pose().t).length2();
            if (dist_sq < min_dist_sq)
            {
                best_id = e->id();
                min_dist_sq = dist_sq;
            }
        }

        if (!best_id.str().empty() && ids.find(best_id) == ids.end())
        {
            ids.insert(best_id);
            associated_ids[i] = best_id;
        }
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update

    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
        const Cluster& cluster = clusters[i];

        ed::UUID id = associated_ids[i];
        if (id.str().empty()) {
            id = ed::Entity::generateID();
            req.addType(id, "suspect");
            req.setType(id, "suspect");

        }

        req.setConvexHullNew(id, cluster.chull, cluster.pose_map, image->getTimestamp());

        // Create and add measurement
        ed::ImageMask mask;
        mask.setSize(image->getDepthImage().cols, image->getDepthImage().rows);
        for(std::vector<unsigned int>::const_iterator it = cluster.pixel_indices.begin(); it != cluster.pixel_indices.end(); ++it)
            mask.addPoint(*it);

        ed::MeasurementPtr m(new ed::Measurement(image, mask, sensor_pose));
        req.addMeasurement(id, m);

        // Set timestamp
        req.setLastUpdateTimestamp(id, image->getTimestamp());
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear unassociated ids


    if (clear_entities_) {
        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape() || !e->has_pose() || e->convexHull().points.empty())
                continue;

            if (ids.find(e->id()) == ids.end())
            {
                req.removeEntity(e->id());
            }
        }
    }



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Visualize

//    cv::imshow("depth", depth / 10);
//    cv::imshow("background", background_depth_ / 10);
//    cv::imshow("diff", diff / 10);
//    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

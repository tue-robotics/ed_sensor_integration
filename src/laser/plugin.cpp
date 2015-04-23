#include "plugin.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
//#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

namespace
{

struct ScanSegment
{
    unsigned int i_start;
    unsigned int i_end;
};

}

// ----------------------------------------------------------------------------------------------------

LaserPlugin::LaserPlugin() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LaserPlugin::~LaserPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::initialize(ed::InitData& init)
{
    std::string laser_topic;
    init.config.value("laser_topic", laser_topic);

    if (init.config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &LaserPlugin::scanCallback, this);

    tf_listener_ = new tf::TransformListener;

    min_segment_size_ = 10;
    world_association_distance_ = 0.2;
    segment_depth_threshold_ = 0.1;

    // Collision check padding
    xy_padding_ = 0;
    z_padding_ = 0;
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    scan_msg_.reset();
    cb_queue_.callAvailable();

    if (!scan_msg_)
        return;

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute laser pose based on TF

    geo::Pose3D sensor_pose;

    if (!tf_listener_->waitForTransform("map", scan_msg_->header.frame_id, scan_msg_->header.stamp, ros::Duration(0.5)))
    {
        ROS_WARN("[ED LASER PLUGIN] Could not get sensor pose");
        return;
    }

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", scan_msg_->header.frame_id, scan_msg_->header.stamp, t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("[ED LASER PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    const std::vector<float>& sensor_ranges = scan_msg_->ranges;

    unsigned int num_beams = sensor_ranges.size();

    if (lrf_model_.getNumBeams() != num_beams)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(scan_msg_->angle_min, scan_msg_->angle_max);
    }

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges(num_beams, 0);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && e->has_pose())
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model_.render(opt, res);
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points
    // and segment the remaining points into clusters

    std::vector<ScanSegment> segments;

    ScanSegment current_segment;
    current_segment.i_start = 0;
    float last_point_dist = sensor_ranges[0];

    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        bool skip = (rs <= 0
                     || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                     || (std::abs(rm - rs) < world_association_distance_)); // If the sensor point is within a certain distance of the world model point, skip it (it is associated)

        if (skip || (i != current_segment.i_start && std::abs(rs - last_point_dist) > segment_depth_threshold_)) // Check the distance of this points to the last.
        {
            // Check if the segment is long enough
            if (i - current_segment.i_start > min_segment_size_)
            {
                current_segment.i_end = i - 1;
                segments.push_back(current_segment);
            }

            if (skip)
                current_segment.i_start = i + 1;
            else
                current_segment.i_start = i;
        }

        if (!skip)
            last_point_dist = rs;
    }

    // - - - - - - - - - - - - - - - - - -
    // Convert the segments to convex hulls, and check for collisions with other convex hulls

    for(std::vector<ScanSegment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const ScanSegment& segment = *it;
        unsigned int segment_size = segment.i_end - segment.i_start + 1;

        cv::Mat_<cv::Vec2f> points(1, segment_size);

        float z_min, z_max;
        for(unsigned int i = 0; i < segment_size; ++i)
        {
            unsigned int j = segment.i_start + i;

            // Calculate the cartesian coordinate of the point in the segment (in sensor frame)
            geo::Vector3 p_sensor = lrf_model_.rayDirections()[j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points(0, i) = cv::Vec2f(p.x, p.y);

            if (i == 0)
            {
                z_min = p.z;
                z_max = p.z;
            }
            else
            {
                z_min = std::min<float>(z_min, p.z);
                z_max = std::max<float>(z_max, p.z);
            }
        }

        geo::Pose3D pose = geo::Pose3D::identity();
        pose.t.z = (z_min + z_max) / 2;

        std::vector<int> chull_indices;
        cv::convexHull(points, chull_indices);

        ed::ConvexHull chull;
        chull.z_min = z_min - pose.t.z;
        chull.z_max = z_max - pose.t.z;

        for(unsigned int i = 0; i < chull_indices.size(); ++i)
        {
            const cv::Vec2f& p_cv = points.at<cv::Vec2f>(chull_indices[i]);
            geo::Vec2f p(p_cv[0], p_cv[1]);

            chull.points.push_back(p);

            pose.t.x += p.x;
            pose.t.y += p.y;
        }

        // Average segment position
        pose.t.x /= chull.points.size();
        pose.t.y /= chull.points.size();

        // Move all points to the pose frame
        for(unsigned int i = 0; i < chull.points.size(); ++i)
        {
            geo::Vec2f& p = chull.points[i];
            p.x -= pose.t.x;
            p.y -= pose.t.y;
        }

        // Calculate normals and edges
        ed::convex_hull::calculateEdgesAndNormals(chull);

        // Check for collisions with convex hulls of existing entities
        bool associated = false;
        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape() || !e->has_pose())
                continue;

            const geo::Pose3D& other_pose = e->pose();
            const ed::ConvexHull& other_chull = e->convexHullNew();

            // Check if the convex hulls collide
            if (ed::convex_hull::collide(other_chull, other_pose.t, chull, pose.t, xy_padding_, z_padding_))
            {
                associated = true;
                break;
            }
        }

        if (!associated)
        {
            ed::UUID id = ed::Entity::generateID();
            req.setPose(id, pose);
            req.setConvexHullNew(id, chull);
        }
    }
}

// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}


ED_REGISTER_PLUGIN(LaserPlugin)

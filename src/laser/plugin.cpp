#include "plugin.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

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

void LaserPlugin::configure(tue::Configuration config)
{
    std::string laser_topic;
    config.value("laser_topic", laser_topic);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &LaserPlugin::scanCallback, this);

    tf_listener_ = new tf::TransformListener;
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

        if (e->shape())
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
        if (rs <= 0)
            continue;

        float rm = model_ranges[i];

        // If the sensor point is behind the world model, skip it
        if (rm > 0 && rs > rm)
            continue;

        // If the sensor point is within a certain distance of the world model point, skip it (it is associated)
        if (std::abs(rm - rs) < 0.2)
            continue;

        // Check the distance of this points to the last. If it exceeds a threshold, end this segment
        if (std::abs(rs - last_point_dist) > 0.1)
        {
            // Check if the segment is long enough
            if (i - current_segment.i_start > 10)
            {
                current_segment.i_end = i - 1;
                segments.push_back(current_segment);
            }

            current_segment.i_end = i;
        }

        last_point_dist = rs;
    }

    // - - - - - - - - - - - - - - - - - -
    // Convert the segments to convex hulls

    unsigned int i_id = 0;
    for(std::vector<ScanSegment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const ScanSegment& segment = *it;

        ed::ConvexHull2D chull;
        chull.center_point = geo::Vector3(0, 0, 0);

        for(unsigned int i = segment.i_start; i <= segment.i_end; ++i)
        {
            // Calculate the cartesian coordinate of the point in the segment (in sensor frame)
            geo::Vector3 p_sensor = lrf_model_.rayDirections()[i] * sensor_ranges[i];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            if (chull.chull.empty())
            {
                chull.min_z = p.z;
                chull.max_z = p.z;
            }
            else
            {
                chull.min_z = std::min(chull.min_z, p.z);
                chull.max_z = std::min(chull.max_z, p.z);
            }

            chull.chull.push_back(pcl::PointXYZ(p.x, p.y, p.z));

            chull.center_point += p;
        }

        chull.center_point = chull.center_point / (segment.i_end - segment.i_start + 1);

        std::stringstream ss;
        ss << "temp-id-" << i_id << std::endl;
        ++i_id;

        req.setType(ss.str(), "bla");
        req.setConvexHull(ss.str(), chull);
    }

//    std::cout << text_ << std::endl;
}


// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}


ED_REGISTER_PLUGIN(LaserPlugin)

#include "plugin.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

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
        float rm = model_ranges[i];

        bool skip = (rs <= 0
                     || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                     || (std::abs(rm - rs) < 0.2)); // If the sensor point is within a certain distance of the world model point, skip it (it is associated)

        if (skip || (i != current_segment.i_start && std::abs(rs - last_point_dist) > 0.1)) // Check the distance of this points to the last.
        {
            // Check if the segment is long enough
            if (i - current_segment.i_start > 10)
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

        ed::ConvexHull2D chull;
        chull.center_point = geo::Vector3(0, 0, 0);

        cv::Mat_<cv::Vec2f> points(1, segment_size);

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
                chull.min_z = p.z;
                chull.max_z = p.z;
            }
            else
            {
                chull.min_z = std::min(chull.min_z, p.z);
                chull.max_z = std::min(chull.max_z, p.z);
            }

            chull.center_point += p;
        }

        chull.center_point = chull.center_point / segment_size;
        chull.center_point.z = (chull.min_z + chull.max_z) / 2;

        std::vector<int> chull_indices;
        cv::convexHull(points, chull_indices);

        for(unsigned int i = 0; i < chull_indices.size(); ++i)
        {
            const cv::Vec2f& p = points.at<cv::Vec2f>(chull_indices[i]);
            chull.chull.push_back(pcl::PointXYZ(p[0], p[1], 0));
        }

        // Check for collisions with convex hulls of existing entities
        bool associated = false;
        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape())
                continue;

            // Multiple measurements per entity BUT 1 entity per measurement
            double overlap_factor;
            if ( ed::helpers::ddp::polygonCollisionCheck(chull, e->convexHull(), overlap_factor) )
            {
                associated = true;
                break;
            }
        }

        if (!associated)
        {
            ed::UUID id = ed::Entity::generateID();
            req.setConvexHull(id, chull);
        }
    }

}


// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}


ED_REGISTER_PLUGIN(LaserPlugin)

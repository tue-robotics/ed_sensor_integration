#include "laser_plugin.h"

#include "ed/laser/entity_update.h"
#include "ed_sensor_integration/association_matrix.h"

#include <ed/convex_hull_calc.h>
#include <ed/entity.h>
#include <ed/io/json_writer.h>
#include <ed/update_request.h>
#include <ed/world_model.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>

#include <geometry_msgs/TransformStamped.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <iostream>


/**
 * copy a ros laserscan message to a standard vector
 *
 * @param[in] scan laserscan message
 * @param[out] sensor_ranges vector with distances ranging 0 and up, not containing null.
 */
void lasermsgToSensorRanges(const sensor_msgs::LaserScan::ConstPtr& scan, std::vector<double>& sensor_ranges)
{
    if (sensor_ranges.size() != scan->ranges.size())
        sensor_ranges.resize(scan->ranges.size());

    for(unsigned int i = 0; i < scan->ranges.size(); ++i)
    {
        double r = scan->ranges[i];
        if (r > scan->range_max)
            sensor_ranges[i] = 0;
        else if (r == r && r > scan->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }
}

LaserPlugin::LaserPlugin()
{
}

LaserPlugin::~LaserPlugin()
{
}

void LaserPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string laser_topic;
    config.value("laser_topic", laser_topic);
    updater_.configure(config);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 3, &LaserPlugin::scanCallback, this);
}

void LaserPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    cb_queue_.callAvailable();

    while(!scan_buffer_.empty())
    {
        sensor_msgs::LaserScan::ConstPtr scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try
        {
            geometry_msgs::TransformStamped gm_sensor_pose;
            gm_sensor_pose = tf_buffer_->lookupTransform("map", scan->header.frame_id, scan->header.stamp);
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert(gm_sensor_pose.transform, sensor_pose);
            update(world, scan, sensor_pose, req);
        }
        catch(tf2::ExtrapolationException& ex)
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE(60, "ED Laserplugin: " << ex.what());
            ROS_DEBUG_STREAM("ED Laserplugin: " << ex.what());
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                geometry_msgs::TransformStamped latest_transform;
                latest_transform = tf_buffer_->lookupTransform("map", scan->header.frame_id, ros::Time(0));

                if (scan_buffer_.front()->header.stamp > latest_transform.header.stamp)
                {
                    // Scan is too new
                    break;
                }
                else
                {
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer_.pop();
                }
            }
            catch(tf2::TransformException& exc)
            {
                scan_buffer_.pop();
            }
        }
        catch(tf2::TransformException& exc)
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin: " << exc.what());
            scan_buffer_.pop();
        }
    }
}

void LaserPlugin::update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                         const geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
{
    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<double> sensor_ranges(scan->ranges.size());
    lasermsgToSensorRanges(scan, sensor_ranges);

    unsigned int num_beams = sensor_ranges.size();

    if (updater_.getNumBeams() != num_beams)
    {
        updater_.configureLaserModel(num_beams, scan->angle_min, scan->angle_max, scan->range_min, scan->range_max);
        updater_.setLaserFrame(scan->header.frame_id);
    }

    updater_.update(world, sensor_ranges, sensor_pose, scan->header.stamp.toSec(), req);
}

// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Received message @ timestamp " << ros::Time::now());

    scan_buffer_.push(msg);
}

ED_REGISTER_PLUGIN(LaserPlugin)

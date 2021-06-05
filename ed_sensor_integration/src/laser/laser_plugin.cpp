#include "laser_plugin.h"
#include <ed/laser/entity_update.h>

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>


/**
 * copy a ros laserscan message to a standard vector
 *
 * @param[in] scan laserscan message
 * @param[out] sensor_ranges vector with ranges
 */
void lasermsgToSensorRanges(const sensor_msgs::LaserScan::ConstPtr& scan, std::vector<float>& sensor_ranges)
{
    if (sensor_ranges.size() != scan->ranges.size())
        sensor_ranges.resize(scan->ranges.size());

    for(unsigned int i = 0; i < scan->ranges.size(); ++i)
    {
        float r = scan->ranges[i];
        if (r > scan->range_max)
            sensor_ranges[i] = r;
        else if (r == r && r > scan->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }
}

LaserPlugin::LaserPlugin() : tf_listener_(0)
{
}

LaserPlugin::~LaserPlugin()
{
    delete tf_listener_;
}

void LaserPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string laser_topic;
    config.value("laser_topic", laser_topic);
    config.value("world_association_distance", world_association_distance_);
    config.value("min_segment_size_pixels", min_segment_size_pixels_);
    config.value("segment_depth_threshold", segment_depth_threshold_);
    config.value("min_cluster_size", min_cluster_size_);
    config.value("max_cluster_size", max_cluster_size_);
    config.value("max_gap_size", max_gap_size_);

    int i_fit_entities = 0;
    config.value("fit_entities", i_fit_entities, tue::config::OPTIONAL);
    fit_entities_ = (i_fit_entities != 0);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 3, &LaserPlugin::scanCallback, this);

    tf_listener_ = new tf::TransformListener;

    //pose_cache.clear();
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
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert(t_sensor_pose, sensor_pose);
            update(world, scan, sensor_pose, req);
        }
        catch(tf::ExtrapolationException& ex)
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin: " << ex.what());
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
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
            catch(tf::TransformException& exc)
            {
                scan_buffer_.pop();
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin: " << exc.what());
            scan_buffer_.pop();
        }
    }
}

void LaserPlugin::update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                         const geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
{
    tue::Timer t_total;
    t_total.start();

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan->ranges.size());
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
    //std::cout << "Received message @ timestamp " << ros::Time::now() << std::endl;

    scan_buffer_.push(msg);
}

ED_REGISTER_PLUGIN(LaserPlugin)

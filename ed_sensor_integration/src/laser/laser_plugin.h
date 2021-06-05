#ifndef ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/laser/updater.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

// TF
#include <tf/transform_listener.h>

#include <geolib/sensors/LaserRangeFinder.h>

// Messages
#include <queue>
#include <sensor_msgs/LaserScan.h>

// Properties
#include "ed/convex_hull.h"

#include <map>

class LaserPlugin : public ed::Plugin
{

public:

    LaserPlugin();

    virtual ~LaserPlugin();

    // initialise plugin
    void initialize(ed::InitData& init);

    // process plugin
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:
    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

    tf::TransformListener* tf_listener_;

    LaserUpdater updater_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief update update the worldmodel based on a novel laserscan message.
     * @param[in] world worldmodel to be updated
     * @param[in] scan laserscan message
     * @param[in] sensor_pose pose of the sensor at the time of the measurement
     * @param[out] req update request
     */
    void update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                const geo::Pose3D& sensor_pose, ed::UpdateRequest& req);
};


#endif

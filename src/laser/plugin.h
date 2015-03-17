#ifndef ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_LASER_PLUGIN_H_

#include <ed/plugin.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

// TF
#include <tf/transform_listener.h>

#include <geolib/sensors/LaserRangeFinder.h>

// Messages
#include <sensor_msgs/LaserScan.h>

// Properties
#include "ed_sensor_integration/properties/convex_hull.h"

// ----------------------------------------------------------------------------------------------------

class LaserPlugin : public ed::Plugin
{

public:

    LaserPlugin();

    virtual ~LaserPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    // PROPERTIES

    ed::PropertyKey<ConvexHull> k_convex_hull_;
    ed::PropertyKey<geo::Pose3D> k_pose_;


    //

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;

    sensor_msgs::LaserScan::ConstPtr scan_msg_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);


    // PARAMETERS

    float xy_padding_;
    float z_padding_;
    int min_segment_size_;
    float world_association_distance_;
    float segment_depth_threshold_;

};


#endif

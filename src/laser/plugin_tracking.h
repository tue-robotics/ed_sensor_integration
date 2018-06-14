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
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <ed_sensor_integration/doorDetection.h>

// Properties
#include "ed/convex_hull.h"

#define MOBIDIK_WIDTH 0.72
#define MOBIDIK_LENGTH 0.81
#define MOBIDIK_MARGIN 0.1

#define INF 10000

// ----------------------------------------------------------------------------------------------------

class LaserPluginTracking : public ed::Plugin
{

public:

    LaserPluginTracking();

    virtual ~LaserPluginTracking();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    //

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;
    
    ros::Publisher door_pub_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                const geo::Pose3D& sensor_pose, ed::UpdateRequest& req);



    // PARAMETERS

    int min_segment_size_pixels_;
    float world_association_distance_;
    float segment_depth_threshold_;
    double min_cluster_size_;
    double max_cluster_size_;
    bool fit_entities_;
    bool check_door_status_;
    float nominal_corridor_width_;

    int max_gap_size_;
    std::map<ed::UUID,geo::Pose3D> pose_cache;
    
    ros::Publisher ObjectMarkers_pub_; 

};


#endif

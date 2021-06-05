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

// Properties
#include "ed/convex_hull.h"

#include <map>

typedef std::vector<unsigned int> ScanSegment;

struct EntityUpdate
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

// ----------------------------------------------------------------------------------------------------

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

    //

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_scan_;

    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

    tf::TransformListener* tf_listener_;

    geo::LaserRangeFinder lrf_model_;

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

    /**
     * configure the LRF model based on a laserscan message
     *
     * @param scan laserscan message
     */
    void configureLaserModel(const sensor_msgs::LaserScan::ConstPtr& scan);

    /**
     * @brief render the worldmodel as would be seen by the lrf.
     * @param[in] sensor_pose pose of the lrf to be modeled in the world frame.
     * @param[in] world worldmodel
     * @param[out] model_ranges ranges of distances as would be seen by an lrf
     */
    void renderWorld(const geo::Pose3D sensor_pose, const ed::WorldModel& world,
                     std::vector<double>& model_ranges);

    /**
     * @brief associate filter sensor information and remove ranges that can be associated with the worldmodel. Leaving only novel data.
     * @param[in] sensor_ranges distances measured by the lrf
     * @param[in] model_ranges distances as predicted by the worldmodel
     * @param[out] filtered_sensor_ranges filtered distances. (associated ranges have value 0.0)
     */
    void associate(const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges, std::vector<float>& filtered_sensor_ranges);

    /** divide the sensor ranges into segments */
    std::vector<ScanSegment> segment(const std::vector<float>& sensor_ranges);

    /** convert a segment of laserdata to a convex hull */
    EntityUpdate segmentToConvexHull(const ScanSegment& segment, const geo::Pose3D sensor_pose, const std::vector<float>& sensor_ranges);

    // PARAMETERS

    int min_segment_size_pixels_;
    float world_association_distance_;
    float segment_depth_threshold_;
    double min_cluster_size_;
    double max_cluster_size_;
    bool fit_entities_;

    int max_gap_size_;
    std::map<ed::UUID,geo::Pose3D> pose_cache;

};


#endif

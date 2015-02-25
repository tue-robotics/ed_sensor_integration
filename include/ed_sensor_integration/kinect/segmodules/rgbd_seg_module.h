#ifndef ed_sensor_integration_kinect_rgbd_seg_module_h_
#define ed_sensor_integration_kinect_rgbd_seg_module_h_

#include <ed/types.h>
#include <ed/rgbd_data.h>
#include <tue/config/configuration.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <visualization_msgs/Marker.h>

namespace edKinect
{

class RGBDSegModule
{

public:

    RGBDSegModule(const ed::TYPE& type) : type_(type)
    {
        ros::NodeHandle nh("~/" + type_);
        vis_marker_pub_ = nh.advertise<visualization_msgs::Marker>("vis_markers",0);
    }

    virtual void process(const ed::RGBDData& rgbd_data, std::vector<ed::PointCloudMaskPtr>& segments) = 0;

    virtual void configure(tue::Configuration config) = 0;

protected:

    ed::TYPE type_;

    ros::Publisher vis_marker_pub_;
    bool visualize_;

};

}

#endif

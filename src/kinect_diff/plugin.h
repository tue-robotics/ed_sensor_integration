#ifndef ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_

#include <queue>

#include <ed/plugin.h>
#include <ed/types.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    cv::Mat background_depth_;


    // COMMUNICATION

    std::string topic_;

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;


    // SERVICES

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;

};

#endif

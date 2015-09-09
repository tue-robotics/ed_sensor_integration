#ifndef ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include "image_buffer.h"

// Services
#include <ros/service_server.h>
#include <ed_sensor_integration/GetImage.h>

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    // Image retrieval

    ImageBuffer image_buffer_;

    rgbd::ImageConstPtr last_image_;

    geo::Pose3D last_sensor_pose_;


    // Communication

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // Services

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_get_image_;

    bool srvGetImage(ed_sensor_integration::GetImage::Request& req, ed_sensor_integration::GetImage::Response& res);

};

#endif

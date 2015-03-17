#ifndef ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>
#include <ed/helpers/image_publisher.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    std::string topic_;

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

};

#endif

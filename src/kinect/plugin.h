#ifndef ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_

#include <ed/plugin.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

};

#endif

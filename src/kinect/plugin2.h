#ifndef ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>


// ----------------------------------------------------------------------------------------------------

class KinectPlugin2 : public ed::Plugin
{

public:

    KinectPlugin2();

    ~KinectPlugin2();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

};

#endif

#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DOWNSAMPLE_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DOWNSAMPLE_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

};

#endif

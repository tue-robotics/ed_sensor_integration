#ifndef ED_SENSOR_INTEGRATION_CLEARER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_CLEARER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

// ----------------------------------------------------------------------------------------------------

class ClearerPlugin : public ed::Plugin
{

public:

    ClearerPlugin();

    ~ClearerPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    double entity_timeout_;

};

#endif

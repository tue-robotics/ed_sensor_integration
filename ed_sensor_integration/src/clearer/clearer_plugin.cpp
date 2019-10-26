#include "clearer_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

// ----------------------------------------------------------------------------------------------------

ClearerPlugin::ClearerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

ClearerPlugin::~ClearerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void ClearerPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;
    config.value("entity_timeout", entity_timeout_);
}

// ----------------------------------------------------------------------------------------------------

void ClearerPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    double current_time = ros::Time::now().toSec();

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        // If the entity is locked, always keep it (don't delete it)
        if (e->hasFlag("locked") || e->shape())
            continue;

        const std::map<std::string, ed::MeasurementConvexHull>& chull_map = e->convexHullMap();

        if (chull_map.empty())
            continue;

        std::vector<std::string> removed_sources;
        for(std::map<std::string, ed::MeasurementConvexHull>::const_iterator it2 = chull_map.begin(); it2 != chull_map.end(); ++it2)
        {
            const ed::MeasurementConvexHull& m = it2->second;
            if (m.timestamp + entity_timeout_ < current_time)
            {
                // too old! Remove this one
                removed_sources.push_back(it2->first);
            }
        }

        if (removed_sources.size() == chull_map.size())
        {
            // All measurement convex hulls are old, so remove the entity!
            req.removeEntity(e->id());
        }
        else
        {
            // Only remove the old measurements
            for(std::vector<std::string>::const_iterator it_source = removed_sources.begin(); it_source != removed_sources.end(); ++it_source)
            {
                req.removeConvexHullNew(e->id(), *it_source);
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(ClearerPlugin)

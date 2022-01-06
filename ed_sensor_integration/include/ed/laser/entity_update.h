#ifndef ED_SENSOR_INTEGRATION_ENTITY_UPDATE_H_
#define ED_SENSOR_INTEGRATION_ENTITY_UPDATE_H_

#include "ed/convex_hull.h"

struct EntityUpdate
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

#endif

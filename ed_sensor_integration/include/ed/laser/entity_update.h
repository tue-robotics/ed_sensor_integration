#ifndef ED_LASER_ENTITY_UPDATE_H_
#define ED_LASER_ENTITY_UPDATE_H_

#include <ed/convex_hull.h>

/**
 * @brief collection structure for laser entities
 */
struct EntityUpdate
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

#endif

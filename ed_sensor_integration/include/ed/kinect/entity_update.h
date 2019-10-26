#ifndef ED_KINECT_ENTITY_UPDATE_H_
#define ED_KINECT_ENTITY_UPDATE_H_

#include <ed/uuid.h>
#include <ed/convex_hull.h>
#include <vector>
#include <geolib/datatypes.h>

struct EntityUpdate
{
    // Association
    bool is_new;
    ed::UUID id;

    // Measurement
    std::vector<unsigned int> pixel_indices;
    std::vector<geo::Vec3> points;

    // Shape
    ed::ConvexHull chull;

    // Pose
    geo::Pose3D pose_map;
};

#endif

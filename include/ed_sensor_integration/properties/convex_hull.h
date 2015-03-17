#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_H_

#include <geolib/datatypes.h>

struct ConvexHull
{
    std::vector<geo::Vec2f> points;
    std::vector<geo::Vec2f> edges;
    std::vector<geo::Vec2f> normals;
    float z_min, z_max;
};

#endif

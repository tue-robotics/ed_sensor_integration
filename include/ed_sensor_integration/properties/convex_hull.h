#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_H_

#include <geolib/datatypes.h>

struct ConvexHull
{
    std::vector<geo::Vec2f> points;
    std::vector<geo::Vec2f> edges;
    std::vector<geo::Vec2f> normals;
    float z_min, z_max;
    float area; // is calculated based on points
    bool complete;

    ConvexHull() : complete(false), area(0) {}

    double height() const { return z_max - z_min; }

    double volume() const { return height() * area; }

};

#endif

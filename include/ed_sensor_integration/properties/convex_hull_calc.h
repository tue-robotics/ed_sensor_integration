#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_CALC_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_CALC_H_

#include "ed_sensor_integration/properties/convex_hull.h"

namespace convex_hull
{

// ----------------------------------------------------------------------------------------------------

void calculateEdgesAndNormals(ConvexHull& c)
{
    c.edges.resize(c.points.size());
    c.normals.resize(c.points.size());

    for(unsigned int i = 0; i < c.points.size(); ++i)
    {
        unsigned int j = (i + 1) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        // Calculate edge
        geo::Vec2f e = p2 - p1;
        c.edges[i] = e;

        // Calculate normal
        c.normals[i] = geo::Vec2f(e.y, -e.x).normalized();
    }
}

// ----------------------------------------------------------------------------------------------------

bool collide(const ConvexHull& c1, const geo::Vector3& pos1,
             const ConvexHull& c2, const geo::Vector3& pos2,
             float xy_padding = 0, float z_padding = 0)
{
    if (c1.points.size() < 3 || c2.points.size() < 3)
        return false;

    float z_diff = pos2.z - pos1.z;

    if (c1.z_max < (c2.z_min + z_diff - 2 * z_padding) || c2.z_max < (c1.z_min + z_diff - 2 * z_padding))
        return false;

    geo::Vec2f pos_diff(pos2.x - pos1.x, pos2.y - pos1.y);

    for(unsigned int i = 0; i < c1.points.size(); ++i)
    {
        const geo::Vec2f& p1 = c1.points[i];
        const geo::Vec2f& n = c1.normals[i];

        // Calculate min and max projection of c1
        float min1 = n.dot(c1.points[0] - p1);
        float max1 = min1;
        for(unsigned int k = 1; k < c1.points.size(); ++k)
        {
            // Calculate projection
            float p = n.dot(c1.points[k] - p1);
            min1 = std::min(min1, p);
            max1 = std::max(max1, p);
        }

        // Apply padding to both sides
        min1 -= xy_padding;
        max1 += xy_padding;

        // Calculate p1 in c2's frame
        geo::Vec2f p1_c2 = p1 - pos_diff;

        // If this bool stays true, there is definitely no collision
        bool no_collision = true;

        // True if projected points are found below c1's bounds
        bool below = false;

        // True if projected points are found above c1's bounds
        bool above = false;

        // Check if c2's points overlap with c1's bounds
        for(unsigned int k = 0; k < c2.points.size(); ++k)
        {
            // Calculate projection on p1's normal
            float p = n.dot(c2.points[k] - p1_c2);

            below = below || (p < max1);
            above = above || (p > min1);

            if (below && above)
            {
                // There is overlap with c1's bound, so we may have a collision
                no_collision = false;
                break;
            }
        }

        if (no_collision)
            // definitely no collision
            return false;
    }

    return true;
}

}

#endif

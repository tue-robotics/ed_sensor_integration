#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_INFO_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_INFO_H_

#include <ed/property_info.h>

#include "ed_sensor_integration/properties/convex_hull.h"

class ConvexHullInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const
    {
        const ConvexHull& c = v.getValue<ConvexHull>();

        w.writeValue("z_min", c.z_min);
        w.writeValue("z_max", c.z_max);

        w.writeArray("points");
        for(unsigned int i = 0; i < c.points.size(); ++i)
        {
            w.addArrayItem();

            const geo::Vec2f& p = c.points[i];
            w.writeValue("x", p.x);
            w.writeValue("y", p.y);

            w.endArrayItem();
        }

        w.endArray();
    }

    bool serializable() const { return true; }

};

#endif

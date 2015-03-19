#ifndef ED_SENSOR_INTEGRATION_CONVEX_HULL_INFO_H_
#define ED_SENSOR_INTEGRATION_CONVEX_HULL_INFO_H_

#include <ed/property_info.h>

#include "ed_sensor_integration/properties/convex_hull.h"

class ConvexHullInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const;

    bool serializable() const { return true; }

};

#endif

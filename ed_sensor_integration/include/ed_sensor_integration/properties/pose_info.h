#ifndef ED_SENSOR_INTEGRATION_POSE_INFO_H_
#define ED_SENSOR_INTEGRATION_POSE_INFO_H_

#include <ed/property_info.h>

class PoseInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const;

    bool serializable() const { return true; }

};

#endif

#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_DEPTH_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_DEPTH_H__

#include <rgbd/types.h>
#include <geolib/datatypes.h>

// Serialization
#include "serializer.h"
#include <rgbd/serialization.h>

// ----------------------------------------------------------------------------------------------------

struct DepthData
{
    rgbd::Image image;   // Will only contain depth data
    geo::Pose3D sensor_pose;
};

// ----------------------------------------------------------------------------------------------------

class DepthDataSerializer : public ArchiveSerializer
{

public:

    void serialize(const bb::Variant& data, tue::serialization::OutputArchive& a)
    {
        const DepthData& d = data.getValue<DepthData>();

        a << d.sensor_pose.t.x;
        a << d.sensor_pose.t.y;
        a << d.sensor_pose.t.z;

        // TODO: rotation

        rgbd::serialize(d.image, a);
    }

    void deserialize(tue::serialization::InputArchive& a, bb::Variant& v)
    {
        DepthData d;

        a >> d.sensor_pose.t.x;
        a >> d.sensor_pose.t.y;
        a >> d.sensor_pose.t.z;

        // TODO: rotation

        rgbd::deserialize(a, d.image);

        v.setValue<DepthData>(d);
    }
};

#endif

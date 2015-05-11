#include "data/depth_data.h"

#include <blackboard/ValueUpdate.h>
#include <blackboard/blackboard.h>

int main(int argc, char **argv)
{

    bb::Serializer* s = new DepthDataSerializer;

    blackboard::ValueUpdate msg;

    {
        DepthData d;
        d.sensor_pose.t = geo::Vector3(1, 2, 3);
        bb::Variant v = d;

        bb::ROSWBytes bytes(msg);
        s->serialize(v, bytes);
    }

    {
        bb::Variant v;
        s->deserialize(bb::ROSRBytes(msg), v);
        const DepthData& d = v.getValue<DepthData>();
        std::cout << d.sensor_pose.t << std::endl;
    }

    return 0;
}

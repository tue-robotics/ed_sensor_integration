#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_MASK_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_MASK_H_

#include <vector>
#include "serializer.h"

struct Mask
{
    std::vector<int> mask;
};

class MaskSerializer: public ArchiveSerializer
{

public:

    void serialize(const bb::Variant& data, tue::serialization::OutputArchive& a)
    {
        const Mask& m = data.getValue<Mask>();

        a << (int)m.mask.size();

        for (int mask_elem: m.mask) {
            a << (int) mask_elem;
        }

    }

    void deserialize(tue::serialization::InputArchive& a, bb::Variant& v)
    {
        Mask m;

        int size;
        a >> size;

        for (int i = 0; i < size; i ++) {
            int elem;

            a >> elem;
            m.mask.push_back(elem);
        }

         v.setValue<Mask>(m);
    }

}

#endif

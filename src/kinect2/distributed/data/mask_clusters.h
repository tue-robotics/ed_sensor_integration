#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_CLUSTERS_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_CLUSTERS_H_

#include <map>
#include <ed/uuid.h>
#include <vector>

#include "serializer.h"

struct MaskClusters
{
    std::map<ed::UUID, std::vector<int> > clusters;
};


class MaskClustersSerializer: public ArchiveSerializer
{

public:

    void serialize(const bb::Variant& data, tue::serialization::OutputArchive& a)
    {
        const MaskClusters& m = data.getValue<MaskClusters>();

        a << (int) m.clusters.size();

        for (auto mask_cl_elem: m.clusters) {
            a << mask_cl_elem.first;
            a << (int) mask_cl_elem.second.size();

            for (auto elem: mask_cl_elem.second) {
                a << elem;
            }
        }
    }

    void deserialize(tue::serialization::InputArchive& a, bb::Variant& v)
    {
        MaskClusters m;

        int map_size;
        a >> map_size;

        for (int i = 0; i < map_size; i ++) {
            int v_size;
            ed::UUID uuid;
            std::vector<int> v;

            a >> uuid;
            a >> v_size;

            for (int j = 0; j < v_size; j ++) {
                int elem;
                a >> elem;

                v.push_back(elem);
            }

            m.clusters[uuid] = v;

        }

        v.setValue<MaskClusters>(m);
    }

}


#endif

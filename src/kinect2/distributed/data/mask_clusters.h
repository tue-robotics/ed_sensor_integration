#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_CLUSTERS_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_CLUSTERS_H_

#include <map>
#include <ed/uuid.h>
#include <vector>

struct MaskClusters
{
    std::map<ed::UUID, std::vector<int> > clusters;
}

#endif

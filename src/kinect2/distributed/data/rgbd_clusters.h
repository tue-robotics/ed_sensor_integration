#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_RGBD_CLUSTERS_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_RGBD_CLUSTERS_H_

#include <map>
#include <ed/uuid.h>
#include <rgbd/types.h>

#include "serializer.h"

struct RGBDClusters
{
    std::map<std::string, rgbd::ImageConstPtr> clusters;
};


#endif

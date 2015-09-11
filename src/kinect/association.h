#ifndef ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_
#define ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_

#include <vector>
#include <ed/types.h>
#include <rgbd/types.h>

class Cluster;

void associateAndUpdate(const ed::WorldModel& world, const std::vector<Cluster>& clusters, const rgbd::ImageConstPtr& image, ed::UpdateRequest& req);

#endif

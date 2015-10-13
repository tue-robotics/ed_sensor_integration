#ifndef ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_
#define ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_

#include <vector>
#include <ed/types.h>
#include <rgbd/types.h>

class EntityUpdate;

void associateAndUpdate(const ed::WorldModel& world, const rgbd::Image& image, std::vector<EntityUpdate>& clusters, ed::UpdateRequest& req);

#endif

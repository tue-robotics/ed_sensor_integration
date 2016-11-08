#ifndef ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_
#define ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_

#include <vector>
#include <ed/types.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <ros/service_client.h>

class EntityUpdate;

void associateAndUpdate(const std::vector<ed::EntityConstPtr>& entities, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                        std::vector<EntityUpdate>& clusters, ed::UpdateRequest& req, ros::ServiceClient& classification_client);

#endif

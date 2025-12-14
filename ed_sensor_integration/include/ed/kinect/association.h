#ifndef ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_
#define ED_SENSOR_INTEGRATION_KINECT_ASSOCATION_L_H_

#include <vector>
#include <ed/types.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>

class EntityUpdate;

void associateAndUpdate(const std::vector<ed::EntityConstPtr>& entities, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                        std::vector<EntityUpdate>& clusters, ed::UpdateRequest& req);

/**
 * @brief Clean up accumulated point clouds for removed entities
 * 
 * Removes stored voxel clouds for entities that were removed from the world model.
 * Call this to prevent memory leaks when entities disappear.
 * 
 * @param removed_entity_ids Vector of entity UUIDs that were removed
 */
void cleanupAccumulatedClouds(const std::vector<ed::UUID>& removed_entity_ids);

#endif

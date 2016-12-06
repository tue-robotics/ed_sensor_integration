#include "ed/kinect/association.h"
#include "ed/kinect/entity_update.h"

#include "ed_sensor_integration/association_matrix.h"
#include "ed_sensor_integration/hungarian_method_association_matrix.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/measurement.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------

const double COST_NOT_OBSERVED = 1;
const double COST_FALSE_DETECTION = 1;

ed_sensor_integration::Assignment associate(const std::vector<ed::EntityConstPtr>& entities,
                                            const std::vector<EntityUpdate>& clusters)
{
    double max_dist = 0.2;
    double max_dist_sq = max_dist*max_dist;

    ed_sensor_integration::Assignment assignment(entities.size(), -1);

    // Create association matrix
    ed_sensor_integration::HungarianMethodAssociationMatrix hung_assoc_matrix(clusters.size(),
                                                                              entities.size(),
                                                                              COST_NOT_OBSERVED,
                                                                              2*max_dist_sq,
                                                                              max_dist_sq);

    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const EntityUpdate& cluster = clusters[i_cluster];
        if ( cluster.chull.points.empty() )
        {
            ROS_ERROR("ed_sensor_integration: association: Skipping empty cluster in association. This should not be happening");
        }

        for (unsigned int i_entity = 0; i_entity < entities.size(); ++i_entity)
        {
            ed::EntityConstPtr e = entities[i_entity];

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            float dx = entity_pose.t.x - cluster.pose_map.t.x;
            float dy = entity_pose.t.y - cluster.pose_map.t.y;
            float dz = 0;

            if (entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose_map.t.z
                    || cluster.chull.z_max + cluster.pose_map.t.z < entity_chull.z_min + entity_pose.t.z)
                // The convex hulls are non-overlapping in z
                dz = entity_pose.t.z - cluster.pose_map.t.z;

            float dist_sq = (dx * dx + dy * dy + dz * dz); // todo: better cost calculation?

            hung_assoc_matrix.setEntry(i_cluster, i_entity, dist_sq);

            if (dist_sq > max_dist_sq)
                dist_sq = 1e9; // set to a high value to avoid this association TODO: magic number

            if (dist_sq > 0)
                hung_assoc_matrix.setEntry(i_cluster, i_entity, dist_sq);
        }
    }

    assignment = hung_assoc_matrix.solve();
    std::stringstream ss;

    for ( size_t i = 0; i < assignment.size(); ++i )
    {
        if ( assignment[i] != -1 )
        {
            ed::EntityConstPtr e = entities[assignment[i]];
            ss << " - " << e->id() << "\n";
        }
    }
    ROS_DEBUG_STREAM("Associated entities:\n " << ss);

    return assignment;
}

void update(const std::vector<ed::EntityConstPtr>& entities,
            const rgbd::ImageConstPtr& image,
            const geo::Pose3D& sensor_pose,
            const ed_sensor_integration::Assignment& assignment,
            std::vector<EntityUpdate>& clusters,
            ed::UpdateRequest& req)
{
    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        EntityUpdate& cluster = clusters[i_cluster];

        // Get the assignment for this cluster
        int i_entity = assignment[i_cluster];

        ed::UUID id;
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;

        if (i_entity == -1)
        {
            // No assignment, so add as new cluster

            // Generate unique ID
            id = ed::Entity::generateID();
            new_chull = cluster.chull;
            new_pose = cluster.pose_map;
            cluster.is_new = true;

            // Update existence probability
            req.setExistenceProbability(cluster.id, 1.0); // TODO magic number

            ROS_DEBUG_STREAM("update: Adding new entity " << id);
        }
        else
        {
            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];

            id = e->id();
            new_chull = cluster.chull;
            new_pose = cluster.pose_map;
            cluster.is_new = false;
            ROS_DEBUG_STREAM("update: updating entity " << id);
        }

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            cluster.id = id;
            cluster.chull = new_chull;
            cluster.pose_map = new_pose;

            req.setConvexHullNew(cluster.id, new_chull, new_pose, image->getTimestamp(), image->getFrameId());
        }
        else
        {
            ROS_ERROR("ed_sensor_integration: association: Skipping empty cluster in update. Again, this should not be happening");
        }

        // Create and add measurement
        ed::ImageMask mask;
        mask.setSize(image->getDepthImage().cols, image->getDepthImage().rows);
        for(std::vector<unsigned int>::const_iterator it = cluster.pixel_indices.begin(); it != cluster.pixel_indices.end(); ++it)
            mask.addPoint(*it);

        ed::MeasurementPtr m(new ed::Measurement(image, mask, sensor_pose));
        req.addMeasurement(id, m);

        // Set timestamp
        req.setLastUpdateTimestamp(id, image->getTimestamp());
    }
}

void associateAndUpdate(const std::vector<ed::EntityConstPtr>& entities,
                        const rgbd::ImageConstPtr& image,
                        const geo::Pose3D& sensor_pose,
                        std::vector<EntityUpdate>& clusters,
                        ed::UpdateRequest& req)
{
    // If there are no clusters, there's no reason to do anything
    if (clusters.empty())
        return;

    // Try to associate clusters with entities
    ed_sensor_integration::Assignment assignment(entities.size(), -1);
    assignment = associate(entities, clusters);

    update(entities, image, sensor_pose, assignment, clusters, req);
}

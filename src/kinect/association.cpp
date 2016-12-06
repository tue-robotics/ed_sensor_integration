#include "ed/kinect/association.h"
#include "ed/kinect/entity_update.h"

#include "ed_sensor_integration/association_matrix.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/measurement.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------

void associateAndUpdate(const std::vector<ed::EntityConstPtr>& entities, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                        std::vector<EntityUpdate>& clusters, ed::UpdateRequest& req)
{
    if (clusters.empty())
        return;

    const cv::Mat& depth = image->getDepthImage();
    rgbd::View view(*image, depth.cols);

    std::vector<int> entities_associated;

    // Create association matrix
    ROS_INFO_STREAM("Nr clusters: " << clusters.size() << ", nr_entities: " << entities.size());
    ed_sensor_integration::AssociationMatrix assoc_matrix(clusters.size());
    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const EntityUpdate& cluster = clusters[i_cluster];

        for (unsigned int i_entity = 0; i_entity < entities.size(); ++i_entity)
        {
            const ed::EntityConstPtr& e = entities[i_entity];

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            float dx = entity_pose.t.x - cluster.pose_map.t.x;
            float dy = entity_pose.t.y - cluster.pose_map.t.y;
            float dz = 0;

            if (entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose_map.t.z
                    || cluster.chull.z_max + cluster.pose_map.t.z < entity_chull.z_min + entity_pose.t.z)
                // The convex hulls are non-overlapping in z
                dz = entity_pose.t.z - cluster.pose_map.t.z;

            float dist_sq = (dx * dx + dy * dy + dz * dz);


            // TODO: better prob calculation
            double prob = 1.0 / (1.0 + 100 * dist_sq);

            double e_max_dist = 0.2;

            if (dist_sq > e_max_dist * e_max_dist)
            {
                prob = 0;
            }

            //                if (entity_chull.complete)
            //                {
            //                    // The idea: if the entity chull is complete, and the cluster chull is significantly taller,
            //                    // the cluster can not be this entity
            //                    if (cluster.chull.height() > 1.5 * entity_chull.height()) // TODO magic number
            //                        prob = 0;
            //                }
            //                if (cluster.chull.complete)
            //                {
            //                    if (entity_chull.height() > 1.5 * cluster.chull.height()) // TODO magic number
            //                        prob = 0;
            //                }

            if (prob > 0)
            {
                ROS_INFO("Entity added to association matrix: %s", e->id().c_str());
                assoc_matrix.setEntry(i_cluster, i_entity, prob);
            }
        }
    } 

    ed_sensor_integration::Assignment assig;
    if (!assoc_matrix.calculateBestAssignment(assig))
    {
        ROS_WARN("Association failed!");
        return;
    }

    entities_associated.resize(entities.size(), -1);

    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        EntityUpdate& cluster = clusters[i_cluster];

        // Get the assignment for this cluster
        int i_entity = assig[i_cluster];

        ed::UUID id;
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;

        if (i_entity == -1)
        {
            // No assignment, so add as new cluster
            new_chull = cluster.chull;
            new_pose = cluster.pose_map;

            // Generate unique ID
            id = ed::Entity::generateID();

            cluster.is_new = true;

            // Update existence probability
            req.setExistenceProbability(id, 1.0); // TODO magic number
        }
        else
        {
            cluster.is_new = false;

//            // Mark the entity as being associated
//            entities_associated[i_entity] = i_cluster;

            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];
            const ed::ConvexHull& entity_chull = e->convexHull();

            id = e->id();

//            if (e->hasFlag("locked"))
//            {
//                // Entity is locked, so don't update
//            }
//            else if (cluster.chull.complete)
//            {
                // Update the entity with the cluster convex hull (completely overriding the previous entity convex hull)
                new_chull = cluster.chull;
                new_pose = cluster.pose_map;
//            }
//            //                else if (entity_chull.complete)
//            //                {
//            //                    // Only update pose
//            //                    new_chull = entity_chull;
//            //                    new_pose = cluster.pose;
//            //                }
//            else
//            {
//                const geo::Pose3D& entity_pose = e->pose();

//                // Calculate the combined z_min and z_max
//                double new_z_min = std::min(cluster.pose.t.z + cluster.chull.z_min, entity_pose.t.z + entity_chull.z_min);
//                double new_z_max = std::max(cluster.pose.t.z + cluster.chull.z_max, entity_pose.t.z + entity_chull.z_max);

//                // Create list of new convex hull points, in MAP frame
//                std::vector<geo::Vec2f> new_points_MAP;

//                // Add the points of the cluster
//                for(std::vector<geo::Vec2f>::const_iterator p_it = cluster.chull.points.begin(); p_it != cluster.chull.points.end(); ++p_it)
//                    new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster.pose.t.x, p_it->y + cluster.pose.t.y));

//                // Add the entity points that are still present in the depth map (or out of view)
//                for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
//                {
//                    geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

//                    // Calculate the 3d coordinate of entity chull points in absolute frame, in the middle of the rib
//                    geo::Vector3 p_rib(p_chull_MAP.x, p_chull_MAP.y, (new_z_min + new_z_max) / 2);

//                    // Transform to the sensor frame
//                    geo::Vector3 p_rib_cam = sensor_pose.inverse() * p_rib;

//                    // Project to image frame
//                    cv::Point2d p_2d = view.getRasterizer().project3Dto2D(p_rib_cam);

//                    // Check if the point is in view, and is not occluded by sensor points
//                    if (p_2d.x > 0 && p_2d.y > 0 && p_2d.x < view.getWidth() && p_2d.y < view.getHeight())
//                    {
//                        // Only add old entity chull point if depth from sensor is now zero, entity point is out of range or depth from sensor is smaller than depth of entity point
//                        float dp = -p_rib_cam.z;
//                        float ds = depth.at<float>(p_2d);
//                        if (ds == 0 || dp > max_sensor_range || dp > ds)
//                            new_points_MAP.push_back(p_chull_MAP);
//                    }
//                    else
//                    {
//                        new_points_MAP.push_back(p_chull_MAP);
//                    }
//                }

//                // And calculate the convex hull of these points
//                ed::convex_hull::create(new_points_MAP, new_z_min, new_z_max, new_chull, new_pose);

//                if (cluster.chull.complete)
//                    new_chull.complete = true;
//            }

//            // Update existence probability
//            double p_exist = e->existenceProbability();
//            req.setExistenceProbability(e->id(), std::min(1.0, p_exist + 0.1)); // TODO: very ugly prob update
        }

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            cluster.id = id;
            cluster.chull = new_chull;
            cluster.pose_map = new_pose;

            req.setConvexHullNew(id, new_chull, new_pose, image->getTimestamp(), image->getFrameId());
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

        // Add measurement
//        req.addMeasurement(id, ed::MeasurementPtr(new ed::Measurement(image, cluster.image_mask, sensor_pose)));
    }
}

#include <ed/laser/updater.h>

#include <iostream>

#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>

/**
 * Calculate an error based on the quality of a fit.
 *
 * @param e entity being fitted
 * @param lrf laser range finder
 * @param rel_pose candidate pose of the entity relative to the laser range finder
 * @param sensor_ranges distances measured by the lrf
 * @param model_ranges predicted measurement distances using the model sans e
 * @param[out] num_model_points number of points used for
 * @return double error
 */
double getFittingError(const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                       const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                       int& num_model_points)
{
    std::vector<double> test_model_ranges = model_ranges;

    // Render the entity with the given relative pose
    geo::LaserRangeFinder::RenderOptions opt;
    opt.setMesh(e.shape()->getMesh(), rel_pose);

    geo::LaserRangeFinder::RenderResult res(test_model_ranges);
    lrf.render(opt, res);

    int n = 0;
    num_model_points = 0;
    double total_error = 0;
    for(unsigned int i = 0; i < test_model_ranges.size(); ++i)
    {
        double ds = sensor_ranges[i];
        double dm = test_model_ranges[i];

        if (ds <= 0) // no sensor data
            continue;

        ++n;

        if (dm <= 0) // no raytrace collision in model
        {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs(ds - dm);
        if (diff < 0.1)
            total_error += diff;
        else
        {
            if (ds > dm)
                total_error += 1;
            else
                total_error += 0.1;
        }

        ++num_model_points;
    }

    return total_error / (n+1); // to be sure to never divide by zero.
}

// retrieve pose from cache, otherwise add pose to cache
geo::Pose3D getPoseFromCache(const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const ed::UUID ID = e.id();
    geo::Pose3D old_pose = e.pose();
    if (pose_cache.find(ID) == pose_cache.end())
    {
        pose_cache[ID] = old_pose;
    }
    else
    {
        old_pose = pose_cache[ID];
    }
    return old_pose;
}

/**
 * estimate the pose of an entity that best describes the sensor data.
 *
 * @param e entity to be fitted
 * @param sensor_pose pose of the sensor in world coordinates
 * @param lrf model of the laser range finder
 * @param sensor_ranges distances measured by the lrf
 * @param model_ranges predicted measurement distances using the model sans e
 * @param x_window window size to sample candidate poses
 * @param x_step step size to sample candidate poses
 * @param y_window window size to sample candidate poses
 * @param y_step step size to sample candidate poses
 * @param yaw_min, yaw_plus window size to sample candidate poses\
 * @param yaw_step step size to sample candidate poses
 * @param pose_cache cache of entity poses
 * @return estimated pose of the entity
 */
geo::Pose3D fitEntity(const ed::Entity& e, const geo::Pose3D& sensor_pose, const geo::LaserRangeFinder& lrf,
                      const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                      float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const geo::Pose3D& old_pose = getPoseFromCache(e, pose_cache);

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    double min_error = 1e6;
    geo::Pose3D best_pose = e.pose();


    for(float dyaw = yaw_min; dyaw <= yaw_plus; dyaw += yaw_step)
    {
        geo::Mat3 rot;
        rot.setRPY(0, 0, dyaw);
        geo::Pose3D test_pose = old_pose;
        test_pose.R = old_pose.R * rot;

        for(float dx = -x_window; dx <= x_window; dx += x_step)
        {
            test_pose.t.x = old_pose.t.x + dx;
            for(float dy = -y_window; dy <= y_window; dy += y_step)
            {
                test_pose.t.y = old_pose.t.y + dy;

                int num_model_points;
                double error = getFittingError(e, lrf, sensor_pose_inv * test_pose, sensor_ranges, model_ranges, num_model_points);

                if (error < min_error && num_model_points >= 3)
                {
                    best_pose = test_pose;
                    min_error = error;
                }
            }
        }
    }
    return best_pose;
}


// check if point p(x,y) is represented in the lrf data. p is expressed relative to the lrf.
bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    int i_beam = lrf.getAngleUpperIndex(x_sensor, y_sensor);
    if (i_beam < 0 || i_beam >= sensor_ranges.size())
        return true; // or actually, we don't know

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2(x_sensor, y_sensor).length() > rs - 0.1;
}

// check if point p is represented in the lrf data. p is expressed relative to the lrf.
bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    return pointIsPresent(p_sensor.x, p_sensor.y, lrf, sensor_ranges);
}

/**
 * @brief findNearbyEntities create a list of entities that are close to detected clusters
 * @param clusters detected clusters
 * @param world worldmodel to get the entities from
 * @return entities that are near the clusters.
 */
std::vector<ed::EntityConstPtr> findNearbyEntities(std::vector<EntityUpdate>& clusters, const ed::WorldModel& world){
    float max_dist = 0.3; //TODO magic number

    // find nearby entities to associate the measurements with
    std::vector<ed::EntityConstPtr> entities;

    if (!clusters.empty())
    {
        geo::Vec2 area_min(clusters[0].pose.t.x, clusters[0].pose.t.y);
        geo::Vec2 area_max(clusters[0].pose.t.x, clusters[0].pose.t.y);
        for (std::vector<EntityUpdate>::const_iterator it = clusters.cbegin(); it != clusters.cend(); ++it)
        {
            const EntityUpdate& cluster = *it;

            area_min.x = std::min(area_min.x, cluster.pose.t.x);
            area_min.y = std::min(area_min.y, cluster.pose.t.y);

            area_max.x = std::max(area_max.x, cluster.pose.t.x);
            area_max.y = std::max(area_max.y, cluster.pose.t.y);
        }

        area_min -= geo::Vec2(max_dist, max_dist);
        area_max += geo::Vec2(max_dist, max_dist);

        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape() || !e->has_pose())
                continue;

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            if (entity_chull.points.empty())
                continue;

            if (entity_pose.t.x < area_min.x || entity_pose.t.x > area_max.x
                    || entity_pose.t.y < area_min.y || entity_pose.t.y > area_max.y)
                continue;

            entities.push_back(e);
        }
    }
    return entities;
}

/**
 * @brief associateSegmentsWithEntities
 * @param[in] clusters measured clusters
 * @param[in] entities entities that may be associated with the clusters
 * @param[in] current_time current time to compare against the last measurement of an entity
 * @param[out] assig assignmentmatrix between clusters and entities.
 * @return whether or not the assignment was successful
 */
bool associateSegmentsWithEntities(std::vector<EntityUpdate>& clusters, const  std::vector<ed::EntityConstPtr>& entities, double current_time, ed_sensor_integration::Assignment& assig)
{
    // Create association matrix
    ed_sensor_integration::AssociationMatrix assoc_matrix(clusters.size());
    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const EntityUpdate& cluster = clusters[i_cluster];

        for (unsigned int i_entity = 0; i_entity < entities.size(); ++i_entity)
        {
            const ed::EntityConstPtr& e = entities[i_entity];

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            float dx = entity_pose.t.x - cluster.pose.t.x;
            float dy = entity_pose.t.y - cluster.pose.t.y;
            float dz = 0;

            if (entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose.t.z
                    || cluster.chull.z_max + cluster.pose.t.z < entity_chull.z_min + entity_pose.t.z)
                // The convex hulls are non-overlapping in z
                dz = entity_pose.t.z - cluster.pose.t.z;

            float dist_sq = (dx * dx + dy * dy + dz * dz);

            // TODO: better prob calculation
            double prob = 1.0 / (1.0 + 100 * dist_sq);

            double dt = current_time - e->lastUpdateTimestamp();

            double e_max_dist = std::max(0.2, std::min(0.5, dt * 10));

            if (dist_sq > e_max_dist * e_max_dist)
                prob = 0;

            if (prob > 0)
                assoc_matrix.setEntry(i_cluster, i_entity, prob);
        }
    }
    return assoc_matrix.calculateBestAssignment(assig);
}

LaserUpdater::LaserUpdater()
{
}

LaserUpdater::~LaserUpdater()
{
}

void LaserUpdater::configure(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    config.value("world_association_distance", world_association_distance_);
    config.value("min_segment_size_pixels", min_segment_size_pixels_);
    config.value("segment_depth_threshold", segment_depth_threshold_);
    config.value("min_cluster_size", min_cluster_size_);
    config.value("max_cluster_size", max_cluster_size_);
    config.value("max_gap_size", max_gap_size_);

    int i_fit_entities = 0;
    config.value("fit_entities", i_fit_entities, tue::config::OPTIONAL);
    fit_entities_ = (i_fit_entities != 0);

    if (config.hasError())
        return;

    //pose_cache.clear();
}

void LaserUpdater::update(const ed::WorldModel& world, const std::vector<float>& sensor_ranges,
                         const geo::Pose3D& sensor_pose, const double timestamp, ed::UpdateRequest& req)
{
    tue::Timer t_total;
    t_total.start();

    uint num_beams = sensor_ranges.size();
    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> filtered_sensor_ranges(num_beams);

    for(unsigned int i = 1; i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if (std::abs(rs - sensor_ranges[i - 1]) > 0.1 && std::abs(rs - sensor_ranges[i + 1]) > 0.1)  // TODO: magic number
        {
            filtered_sensor_ranges[i] = sensor_ranges[i - 1];
        }
        else
        {
            filtered_sensor_ranges[i] = sensor_ranges[i];
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    std::cout << "updating laserscan" << std::endl;
    std::vector<double> model_ranges(num_beams, 0);
    renderWorld(sensor_pose, world, model_ranges);

    // - - - - - - - - - - - - - - - - - -
    // Fit the doors

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    if (fit_entities_)
    {
        std::cout << "Fitting!" << std::endl;

        for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;
            //std::cout << e->type() << std::endl;

            if (!e->shape() || !e->has_pose())
                continue;

            geo::Pose3D e_pose_SENSOR = sensor_pose_inv * e->pose();

            // If not in sensor view, continue
            if (e_pose_SENSOR.t.length2() > 5.0 * 5.0 || e_pose_SENSOR.t.x < 0)
                continue;

            if (e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right"))
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity(*e, sensor_pose, lrf_model_, filtered_sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache);
                req.setPose(e->id(), new_pose);

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * new_pose);

                geo::LaserRangeFinder::RenderResult res(model_ranges);
                lrf_model_.render(opt, res);
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    std::vector<float> associated_ranges(num_beams);
    associate(filtered_sensor_ranges, model_ranges, associated_ranges);

    // - - - - - - - - - - - - - - - - - -
    // Segment the remaining points into clusters

    std::vector<ScanSegment> segments = segment(associated_ranges);

    // - - - - - - - - - - - - - - - - - -
    // Convert the segments to convex hulls, and check for collisions with other convex hulls

    std::vector<EntityUpdate> clusters;

    for(std::vector<ScanSegment>::const_iterator it = segments.cbegin(); it != segments.cend(); ++it)
    {
        const ScanSegment& segment = *it;
        clusters.push_back(segmentToConvexHull(segment, sensor_pose, associated_ranges));
    }

    // Create selection of world model entities that could associate
    std::vector<ed::EntityConstPtr> entities = findNearbyEntities(clusters, world);
    ed_sensor_integration::Assignment assig;
    if (!associateSegmentsWithEntities(clusters, entities, timestamp, assig))
    {
        std::cout << "WARNING: Association failed!" << std::endl;
        return;
    }

    //std::vector<int> entities_associated(entities.size(), -1); // used to clear unaccociated entities

    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const EntityUpdate& cluster = clusters[i_cluster];

        // Get the assignment for this cluster
        int i_entity = assig[i_cluster];

        ed::UUID id;
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;

        if (i_entity == -1)
        {
            // No assignment, so add as new cluster
            new_chull = cluster.chull;
            new_pose = cluster.pose;

            // Generate unique ID
            id = ed::Entity::generateID().str() + "-laser";

            // Update existence probability
            req.setExistenceProbability(id, 1.0); // TODO magic number
        }
        else
        {
            // Mark the entity as being associated
            //entities_associated[i_entity] = i_cluster;

            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];

            if (!e->hasFlag("locked"))
            {
                new_chull = cluster.chull;
                new_pose = cluster.pose;
            }

            // Update existence probability
            double p_exist = e->existenceProbability();
            req.setExistenceProbability(e->id(), std::min(1.0, p_exist + 0.1)); // TODO: very ugly prob update

            id = e->id();
        }

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            req.setConvexHullNew(id, new_chull, new_pose, timestamp, lrf_frame_);

            // --------------------------
            // Temp for RoboCup 2015; todo: remove after

            if (!cluster.flag.empty())
                req.setFlag(id, cluster.flag);

            // --------------------------
        }

        // Set timestamp
        req.setLastUpdateTimestamp(id, timestamp);
    }

    // - - - - - - - - - - - - - - - - - -
    // Clear unassociated entities in view

    //    for(unsigned int i = 0; i < entities_associated.size(); ++i)
    //    {
    //        const ed::EntityConstPtr& e = entities[i];

    //        // If the entity is associated, skip it
    //        if (entities_associated[i] >= 0)
    //            continue;

    //        const geo::Pose3D& pose = e->pose();

    //        // Transform to sensor frame
    //        geo::Vector3 p = sensor_pose.inverse() * pose.t;

    //        if (!pointIsPresent(p, lrf_model_, sensor_ranges))
    //        {
    //            double p_exist = e->existenceProbability();
    //            if (p_exist < 0.3) // TODO: magic number
    //                req.removeEntity(e->id());
    //            else
    //            {
    //                req.setExistenceProbability(e->id(), std::max(0.0, p_exist - 0.1));  // TODO: very ugly prob update
    //            }
    //        }
    //    }

    //std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void LaserUpdater::renderWorld(const geo::Pose3D sensor_pose, const ed::WorldModel& world, std::vector<double>& model_ranges)
{
    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && e->has_pose() && !(e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right")))
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model_.render(opt, res);
        }
    }
}

void LaserUpdater::associate(const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges, std::vector<float>& filtered_sensor_ranges){
    for(unsigned int i = 0; i < sensor_ranges.size(); ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        filtered_sensor_ranges[i] = rs;
        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
            filtered_sensor_ranges[i] = 0;
    }
}

std::vector<ScanSegment> LaserUpdater::segment(const std::vector<float>& sensor_ranges)
{
    std::vector<ScanSegment> segments;
    int num_beams = sensor_ranges.size();

    // Find first valid value
    ScanSegment current_segment;
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        if (sensor_ranges[i] > 0)
        {
            current_segment.push_back(i);
            break;
        }
    }

    if (current_segment.empty()) // no residual point cloud
    {
        return segments;
    }

    int gap_size = 0;

    for(unsigned int i = current_segment.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_ || i == num_beams - 1)
        {
            // Found a gap or at final reading
            ++gap_size;

            if (gap_size >= max_gap_size_ || i == num_beams - 1)
            {
                i = current_segment.back() + 1;

                if (current_segment.size() >= min_segment_size_pixels_)
                {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for(unsigned int k = 0; k < current_segment.size(); ++k)
                    {
                        geo::Vector3 p = lrf_model_.rayDirections()[current_segment[k]] * sensor_ranges[current_segment[k]];

                        if (k == 0)
                        {
                            seg_min = geo::Vec2(p.x, p.y);
                            seg_max = geo::Vec2(p.x, p.y);
                        }
                        else
                        {
                            seg_min.x = std::min(p.x, seg_min.x);
                            seg_min.y = std::min(p.y, seg_min.y);
                            seg_max.x = std::max(p.x, seg_max.x);
                            seg_max.y = std::max(p.y, seg_max.y);
                        }
                    }

                    geo::Vec2 bb = seg_max - seg_min;
                    if ((bb.x > min_cluster_size_ || bb.y > min_cluster_size_) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_)
                        segments.push_back(current_segment);
                }

                current_segment.clear();

                // Find next good value
                while(sensor_ranges[i] == 0 && i < num_beams)
                    ++i;

                current_segment.push_back(i);
            }
        }
        else
        {
            gap_size = 0;
            current_segment.push_back(i);
        }
    }
    return segments;
}

EntityUpdate LaserUpdater::segmentToConvexHull(const ScanSegment& segment, const geo::Pose3D sensor_pose, const std::vector<float>& sensor_ranges)
{
    unsigned int segment_size = segment.size();

    std::vector<geo::Vec2f> points(segment_size);

    float z_min, z_max;
    for(unsigned int i = 0; i < segment_size; ++i)
    {
        unsigned int j = segment[i];

        // Calculate the cartesian coordinate of the point in the segment (in sensor frame)
        geo::Vector3 p_sensor = lrf_model_.rayDirections()[j] * sensor_ranges[j];

        // Transform to world frame
        geo::Vector3 p = sensor_pose * p_sensor;

        // Add to cv array
        points[i] = geo::Vec2f(p.x, p.y);

        if (i == 0)
        {
            z_min = p.z;
            z_max = p.z;
        }
        else
        {
            z_min = std::min<float>(z_min, p.z);
            z_max = std::max<float>(z_max, p.z);
        }
    }

    EntityUpdate cluster;

    cluster.pose = geo::Pose3D::identity();
    ed::convex_hull::create(points, z_min, z_max, cluster.chull, cluster.pose);

    // --------------------------
    // Temp for RoboCup 2016; todo: remove after

    // Determine the cluster size
    geo::Vec2f diff = points.back() - points.front();
    float size_sq = diff.length2();
    if (size_sq > 0.35 * 0.35 && size_sq < 0.8 * 0.8)
        cluster.flag = "possible_human";

    // --------------------------
    return cluster;
}

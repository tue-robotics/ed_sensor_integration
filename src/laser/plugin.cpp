#include "plugin.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
//#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>

namespace
{

typedef std::vector<unsigned int> ScanSegment;

struct Cluster
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    int i_beam = lrf.getAngleUpperIndex(x_sensor, y_sensor);
    if (i_beam < 0 || i_beam >= sensor_ranges.size())
        return true; // or actually, we don't know

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2(x_sensor, y_sensor).length() > rs - 0.1;
}

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    return pointIsPresent(p_sensor.x, p_sensor.y, lrf, sensor_ranges);
}

}

// ----------------------------------------------------------------------------------------------------

LaserPlugin::LaserPlugin() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LaserPlugin::~LaserPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::initialize(ed::InitData& init)
{
    std::string laser_topic;
    init.config.value("laser_topic", laser_topic);

    if (init.config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 10, &LaserPlugin::scanCallback, this);

    tf_listener_ = new tf::TransformListener;

    min_segment_size_pixels_ = 10;
    world_association_distance_ = 0.2;
    segment_depth_threshold_ = 0.2;

    min_cluster_size_ = 0.2;
    max_cluster_size_ = 1.0;

    max_gap_size_ = 10;
}

// ----------------------------------------------------------------------------------------------------

void LaserPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    scan_msg_.reset();
    cb_queue_.callAvailable();

    if (!scan_msg_)
        return;

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute laser pose based on TF

    geo::Pose3D sensor_pose;

    if (!tf_listener_->waitForTransform("map", scan_msg_->header.frame_id, scan_msg_->header.stamp, ros::Duration(0.5)))
    {
        ROS_WARN("[ED LASER PLUGIN] Could not get sensor pose");
        return;
    }

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", scan_msg_->header.frame_id, scan_msg_->header.stamp, t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("[ED LASER PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    tue::Timer t_total;
    t_total.start();

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan_msg_->ranges.size());
    for(unsigned int i = 0; i < scan_msg_->ranges.size(); ++i)
    {
        float r = scan_msg_->ranges[i];
        if (r > scan_msg_->range_max)
            sensor_ranges[i] = r;
        else if (r == r && r > scan_msg_->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }

    unsigned int num_beams = sensor_ranges.size();

    if (lrf_model_.getNumBeams() != num_beams)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(scan_msg_->angle_min, scan_msg_->angle_max);
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for(unsigned int i = 1; i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if (std::abs(rs - sensor_ranges[i - 1]) > 0.1 && std::abs(rs - sensor_ranges[i + 1]) > 0.1)  // TODO: magic number
        {
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }


    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges(num_beams, 0);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && e->has_pose())
        {
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model_.render(opt, res);
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
            sensor_ranges[i] = 0;
    }

    // - - - - - - - - - - - - - - - - - -
    // Segment the remaining points into clusters

    std::vector<ScanSegment> segments;

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

    if (current_segment.empty())
    {
        //std::cout << "No residual point cloud!" << std::endl;
        return;
    }

    int gap_size = 0;

    for(unsigned int i = current_segment.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_)
        {
            // Found a gap
            ++gap_size;

            if (gap_size >= max_gap_size_)
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

    // - - - - - - - - - - - - - - - - - -
    // Convert the segments to convex hulls, and check for collisions with other convex hulls

    std::vector<Cluster> clusters;

    for(std::vector<ScanSegment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const ScanSegment& segment = *it;
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

        clusters.push_back(Cluster());
        Cluster& cluster = clusters.back();

        cluster.pose = geo::Pose3D::identity();
        ed::convex_hull::create(points, z_min, z_max, cluster.chull, cluster.pose);

        // --------------------------
        // Temp for RoboCup 2015; todo: remove after

        // Determine the cluster size
        geo::Vec2f diff = points.back() - points.front();
        float size_sq = diff.length2();
        if (size_sq > 0.35 * 0.35 && size_sq < 0.8 * 0.8)
            cluster.flag = "possible_human";

        // --------------------------
    }

    // Create selection of world model entities that could associate

    float max_dist = 0.3;

    std::vector<ed::EntityConstPtr> entities;

    if (!clusters.empty())
    {
        geo::Vec2 area_min(clusters[0].pose.t.x, clusters[0].pose.t.y);
        geo::Vec2 area_max(clusters[0].pose.t.x, clusters[0].pose.t.y);
        for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
        {
            const Cluster& cluster = *it;

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

//            if (e->existenceProbability() < 0.5 && scan_msg_->header.stamp.toSec() - e->lastUpdateTimestamp() > 1.0) // TODO: magic numbers
//            {
//                req.removeEntity(e->id());
//                continue;
//            }

            if (entity_pose.t.x < area_min.x || entity_pose.t.x > area_max.x
                    || entity_pose.t.y < area_min.y || entity_pose.t.y > area_max.y)
                continue;

            entities.push_back(e);
        }
    }

    // Create association matrix
    ed_sensor_integration::AssociationMatrix assoc_matrix(clusters.size());
    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const Cluster& cluster = clusters[i_cluster];

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

            double dt = scan_msg_->header.stamp.toSec() - e->lastUpdateTimestamp();

            double e_max_dist = std::max(0.2, std::min(0.5, dt * 10));

            if (dist_sq > e_max_dist * e_max_dist)
                prob = 0;

            if (prob > 0)
                assoc_matrix.setEntry(i_cluster, i_entity, prob);
        }
    }

    ed_sensor_integration::Assignment assig;
    if (!assoc_matrix.calculateBestAssignment(assig))
    {
        std::cout << "WARNING: Association failed!" << std::endl;
        return;
    }

    std::vector<int> entities_associated(entities.size(), -1);

    for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
    {
        const Cluster& cluster = clusters[i_cluster];

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
            entities_associated[i_entity] = i_cluster;

            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];
//            const ed::ConvexHull& entity_chull = e->convexHullNew();
//            const geo::Pose3D& entity_pose = e->pose();

//            std::vector<geo::Vec2f> new_points_MAP;

//            // Add the points of the cluster
//            for(std::vector<geo::Vec2f>::const_iterator p_it = cluster.chull.points.begin(); p_it != cluster.chull.points.end(); ++p_it)
//                new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster.pose.t.x, p_it->y + cluster.pose.t.y));

//            // Add the entity points that are still present in the depth map (or out of view)
//            for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
//            {
//                geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

//                geo::Vector3 p = sensor_pose.inverse() * geo::Vector3(p_chull_MAP.x, p_chull_MAP.y, entity_pose.t.z);

//                if (pointIsPresent(p, lrf_model_, sensor_ranges))
//                {
//                    new_points_MAP.push_back(p_chull_MAP);
//                }
//            }

//            double new_z_min = cluster.chull.z_min;
//            double new_z_max = cluster.chull.z_max;

//            // And calculate the convex hull of these points
//            ed::convex_hull::create(new_points_MAP, new_z_min, new_z_max, new_chull, new_pose);

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
            req.setConvexHullNew(id, new_chull, new_pose, scan_msg_->header.stamp.toSec(), scan_msg_->header.frame_id);

            // --------------------------
            // Temp for RoboCup 2015; todo: remove after

            if (!cluster.flag.empty())
                req.setFlag(id, cluster.flag);

            // --------------------------
        }

        // Set timestamp
        req.setLastUpdateTimestamp(id, scan_msg_->header.stamp.toSec());
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

//    std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl;

}

// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}


ED_REGISTER_PLUGIN(LaserPlugin)

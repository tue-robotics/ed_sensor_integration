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

namespace
{

typedef std::vector<unsigned int> ScanSegment;

struct Cluster
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
};

}

// ----------------------------------------------------------------------------------------------------

void convertConvexHull(const ed::ConvexHull& c, const geo::Pose3D& pose, ed::ConvexHull2D& c2)
{
    c2.min_z = c.z_min + pose.t.z;
    c2.max_z = c.z_max + pose.t.z;
    c2.center_point = pose.t;

    c2.chull.resize(c.points.size());
    for(unsigned int i = 0; i < c.points.size(); ++i)
        c2.chull.points[i] = pcl::PointXYZ(c.points[i].x + pose.t.x, c.points[i].y + pose.t.y, 0);
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

    min_segment_size_ = 10;
    world_association_distance_ = 0.2;
    segment_depth_threshold_ = 0.3;

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

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan_msg_->ranges.size());
    for(unsigned int i = 0; i < scan_msg_->ranges.size(); ++i)
    {
        float r = scan_msg_->ranges[i];
        if (r == r && r > scan_msg_->range_min && r < scan_msg_->range_max)
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
        std::cout << "No residual point cloud!" << std::endl;
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

                if (current_segment.size() >= min_segment_size_)
                    segments.push_back(current_segment);

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
            const ed::ConvexHull& entity_chull = e->convexHullNew();

            if (entity_chull.points.empty())
                continue;

            if (e->existenceProbability() < 0.5 && scan_msg_->header.stamp.toSec() - e->lastUpdateTimestamp() > 1.0) // TODO: magic numbers
            {
                req.removeEntity(e->id());
                continue;
            }

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
            const ed::ConvexHull& entity_chull = e->convexHullNew();

            double dx = entity_pose.t.x - cluster.pose.t.x;
            double dy = entity_pose.t.y - cluster.pose.t.y;

            double dist_sq = (dx * dx) + (dy * dy);

            // TODO: better prob calculation
            double prob = 1.0 / (1.0 + 100 * dist_sq);

            assoc_matrix.setEntry(i_cluster, i_entity, prob);
        }
    }

    ed_sensor_integration::Assignment assig;
    if (!assoc_matrix.calculateBestAssignment(assig))
    {
        std::cout << "WARNING: Association failed!" << std::endl;
        return;
    }

    std::vector<int> entities_associated;

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
            id = ed::Entity::generateID();

            // Update existence probability
            req.setExistenceProbability(id, 0.2); // TODO magic number
        }
        else
        {
            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];
            const ed::ConvexHull& entity_chull = e->convexHullNew();

            new_chull = cluster.chull;
            new_pose = cluster.pose;

            // Update existence probability
            double p_exist = e->existenceProbability();
            req.setExistenceProbability(e->id(), std::min(1.0, p_exist + 0.1)); // TODO: very ugly prob update

            id = e->id();
        }

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            req.setConvexHullNew(id, new_chull);

            // Set old chull (is used in other plugins, e.g. navigation)
            ed::ConvexHull2D chull_old;
            convertConvexHull(new_chull, new_pose, chull_old);
            req.setConvexHull(id, chull_old);
        }

        req.setPose(id, new_pose);

        // Set timestamp
        req.setLastUpdateTimestamp(id, scan_msg_->header.stamp.toSec());
    }
}

// ----------------------------------------------------------------------------------------------------


void LaserPlugin::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_msg_ = msg;
}


ED_REGISTER_PLUGIN(LaserPlugin)

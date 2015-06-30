#include "plugin.h"

#include <geolib/datatypes.h>
#include <geolib/ros/tf_conversions.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/integral_image_normal.h>

// Rendering
#include <ed/world_model/transform_crawler.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <pcl/filters/filter.h>

#include <ed/update_request.h>

#include <ed/convex_hull_calc.h>

#include <ed/measurement.h>
#include <ed/mask.h>

// Visualization
#include "visualization.h"

#include <ed/error_context.h>

// Association
#include "ed_sensor_integration/association_matrix.h"

// Meshing
#include "ed_sensor_integration/meshing.h"
#include <geolib/serialization.h>

// ----------------------------------------------------------------------------------------------------

geo::Vector3 min(const geo::Vector3& p1, const geo::Vector3& p2)
{
    return geo::Vector3(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
}

// ----------------------------------------------------------------------------------------------------

geo::Vector3 max(const geo::Vector3& p1, const geo::Vector3& p2)
{
    return geo::Vector3(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
}

// ----------------------------------------------------------------------------------------------------

//void convertConvexHull(const ed::ConvexHull& c, const geo::Pose3D& pose, ed::ConvexHull2D& c2)
//{
//    c2.min_z = c.z_min + pose.t.z;
//    c2.max_z = c.z_max + pose.t.z;
//    c2.center_point = pose.t;

//    c2.chull.resize(c.points.size());
//    for(unsigned int i = 0; i < c.points.size(); ++i)
//        c2.chull.points[i] = pcl::PointXYZ(c.points[i].x + pose.t.x, c.points[i].y + pose.t.y, 0);
//}

// ----------------------------------------------------------------------------------------------------

void associateAndUpdate(const ed::WorldModel& world, const std::vector<Cluster>& clusters, const rgbd::ImageConstPtr& image,
                        const geo::Pose3D& sensor_pose, double max_sensor_range, ed::UpdateRequest& req)
{
    ed::ErrorContext errc("Convex hull assocation");

    if (clusters.empty())
        return;

    const cv::Mat& depth = image->getDepthImage();
    rgbd::View view(*image, depth.cols);

    float max_dist = 0.3;

    // Create selection of world model entities that could associate

    std::vector<ed::EntityConstPtr> entities;
    std::vector<int> entities_associated;

    geo::Vector3 area_min = clusters[0].pose.t;
    geo::Vector3 area_max = clusters[0].pose.t;
    for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        const Cluster& cluster = *it;
        area_min = min(area_min, cluster.pose.t);
        area_max = max(area_max, cluster.pose.t);
    }

    area_min -= geo::Vector3(max_dist, max_dist, max_dist);
    area_max += geo::Vector3(max_dist, max_dist, max_dist);

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
                || entity_pose.t.y < area_min.y || entity_pose.t.y > area_max.y
                || entity_pose.t.z < area_min.z || entity_pose.t.z > area_max.z)
        {
            continue;
        }

        entities.push_back(e);
    }

    //        std::cout << "Clusters: " << clusters.size() << ", entities: " <<  entities.size() << std::endl;

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

            double e_max_dist = 0.2;

            if (dist_sq > e_max_dist * e_max_dist)
                prob = 0;

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
                assoc_matrix.setEntry(i_cluster, i_entity, prob);
        }
    }

    ed_sensor_integration::Assignment assig;
    if (!assoc_matrix.calculateBestAssignment(assig))
    {
        std::cout << "WARNING: Association failed!" << std::endl;
        return;
    }

    entities_associated.resize(entities.size(), -1);

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
            // Mark the entity as being associated
            entities_associated[i_entity] = i_cluster;

            // Update the entity
            const ed::EntityConstPtr& e = entities[i_entity];
            const ed::ConvexHull& entity_chull = e->convexHull();

            id = e->id();

            if (cluster.chull.complete)
            {
                // Update the entity with the cluster convex hull (completely overriding the previous entity convex hull)
                new_chull = cluster.chull;
                new_pose = cluster.pose;
            }
            //                else if (entity_chull.complete)
            //                {
            //                    // Only update pose
            //                    new_chull = entity_chull;
            //                    new_pose = cluster.pose;
            //                }
            else
            {
                const geo::Pose3D& entity_pose = e->pose();

                // Calculate the combined z_min and z_max
                double new_z_min = std::min(cluster.pose.t.z + cluster.chull.z_min, entity_pose.t.z + entity_chull.z_min);
                double new_z_max = std::max(cluster.pose.t.z + cluster.chull.z_max, entity_pose.t.z + entity_chull.z_max);

                // Create list of new convex hull points, in MAP frame
                std::vector<geo::Vec2f> new_points_MAP;

                // Add the points of the cluster
                for(std::vector<geo::Vec2f>::const_iterator p_it = cluster.chull.points.begin(); p_it != cluster.chull.points.end(); ++p_it)
                    new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster.pose.t.x, p_it->y + cluster.pose.t.y));

                // Add the entity points that are still present in the depth map (or out of view)
                for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
                {
                    geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

                    // Calculate the 3d coordinate of entity chull points in absolute frame, in the middle of the rib
                    geo::Vector3 p_rib(p_chull_MAP.x, p_chull_MAP.y, (new_z_min + new_z_max) / 2);

                    // Transform to the sensor frame
                    geo::Vector3 p_rib_cam = sensor_pose.inverse() * p_rib;

                    // Project to image frame
                    cv::Point2d p_2d = view.getRasterizer().project3Dto2D(p_rib_cam);

                    // Check if the point is in view, and is not occluded by sensor points
                    if (p_2d.x > 0 && p_2d.y > 0 && p_2d.x < view.getWidth() && p_2d.y < view.getHeight())
                    {
                        // Only add old entity chull point if depth from sensor is now zero, entity point is out of range or depth from sensor is smaller than depth of entity point
                        float dp = -p_rib_cam.z;
                        float ds = depth.at<float>(p_2d);
                        if (ds == 0 || dp > max_sensor_range || dp > ds)
                            new_points_MAP.push_back(p_chull_MAP);
                    }
                    else
                    {
                        new_points_MAP.push_back(p_chull_MAP);
                    }
                }

                // And calculate the convex hull of these points
                ed::convex_hull::create(new_points_MAP, new_z_min, new_z_max, new_chull, new_pose);

                if (cluster.chull.complete)
                    new_chull.complete = true;
            }

            // Update existence probability
            double p_exist = e->existenceProbability();
            req.setExistenceProbability(e->id(), std::min(1.0, p_exist + 0.1)); // TODO: very ugly prob update
        }

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            req.setConvexHullNew(id, new_chull, new_pose, image->getTimestamp(), image->getFrameId());
        }

        // Set timestamp
        req.setLastUpdateTimestamp(id, image->getTimestamp());

        // Add measurement
        req.addMeasurement(id, ed::MeasurementPtr(new ed::Measurement(image, cluster.image_mask, sensor_pose)));
    }
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin() : tf_listener_(0), debug_(false), continuous_(true)
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    if (config.value("topic", topic_))
    {
        std::cout << "[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic_ << "'." << std::endl;
        kinect_client_.intialize(topic_);
    }

    config.value("max_correspondence_distance", segmenter_.association_correspondence_distance_);
    config.value("downsample_factor", segmenter_.downsample_factor_);
    config.value("max_range", segmenter_.max_range_);

    // Get parameter that determines if perception should be run continuously or not
    int int_continuous = 0;
    if (config.value("continuous", int_continuous))
        continuous_ = int_continuous;

    if (config.value("debug", debug_, tue::OPTIONAL))
    {
        if (debug_)
            std::cout << "[ED KINECT PLUGIN] Debug print statements on" << std::endl;
    }

    tf_listener_ = new tf::TransformListener;

    xy_padding_ = 0.1;
    z_padding_ = 0.1;
    border_padding_ = 0.1;

    // Initialize image publishers for visualization
    viz_sensor_normals_.initialize("kinect/viz/sensor_normals");
    viz_model_normals_.initialize("kinect/viz/model_normals");
    viz_clusters_.initialize("kinect/viz/clusters");
    viz_update_req_.initialize("kinect/viz/update_request");
    viz_model_render_.initialize("kinect/viz/depth_render");

    // Initialize lock entity server
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_segment_ = nh.advertiseService("kinect/segment", &KinectPlugin::srvSegment, this);
    srv_lock_entities_ = nh.advertiseService("kinect/lock_entities", &KinectPlugin::srvLockEntities, this);
    srv_mesh_entity_in_view_ = nh.advertiseService("kinect/mesh_entity_in_view", &KinectPlugin::srvMeshEntityInView, this);
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();

    // - - - - - - - - - - - - - - - - - -

    if (!continuous_)
        return;

    tue::Timer t_total;
    t_total.start();

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_.nextImage();
    if (rgbd_image)
        image_buffer_.push(rgbd_image);

    if (image_buffer_.empty())
        return;

    rgbd_image = image_buffer_.front();


    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    geo::Pose3D sensor_pose;

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
        image_buffer_.pop();

    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
            {
                image_buffer_.pop();
                ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(rgbd_image->getTimestamp()));
            }

            return;
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            return;
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<Cluster> clusters;
    segmenter_.segment(world, *rgbd_image, sensor_pose, clusters);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    associateAndUpdate(world, clusters, rgbd_image, sensor_pose, max_range_, req);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



    //    if (!mesh_request_.id.empty())
    //    {
    //        const Cluster* biggest_cluster = 0;

    //        for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    //        {
    //            const Cluster& cluster = *it;
    //            if (!biggest_cluster || cluster.pixels.size() > biggest_cluster->pixels.size())
    //            {
    //                biggest_cluster = &cluster;
    //            }
    //        }

    //        if (biggest_cluster && biggest_cluster->mask.size() > 500) // TODO: magic number
    //        {
    //            cv::Mat masked_depth_img(depth.rows, depth.cols, CV_32FC1, cv::Scalar(0));
    //            for(unsigned int i = 0; i < biggest_cluster->mask.size(); ++i)
    //                masked_depth_img.at<float>(biggest_cluster->mask[i]) = depth.at<float>(biggest_cluster->mask[i]);

    //            geo::Mesh mesh;
    //            ed_sensor_integration::depthImageToMesh(masked_depth_img, view.getRasterizer(), 0.05, mesh);

    //            // Transform mesh to map frame
    //            mesh = mesh.getTransformed(sensor_pose);

    //            // Determine bounding box
    //            const std::vector<geo::Vector3>& points = mesh.getPoints();
    //            geo::Vector3 points_min = points.front();
    //            geo::Vector3 points_max = points.front();
    //            for(std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    //            {
    //                points_min = min(points_min, *it);
    //                points_max = max(points_max, *it);
    //            }

    //            // Determine origin
    //            geo::Vector3 origin = (points_min + points_max) / 2;

    //            geo::Transform transform(geo::Matrix3::identity(), -origin);
    //            mesh = mesh.getTransformed(transform);

    //            geo::ShapePtr shape(new geo::Shape);
    //            shape->setMesh(mesh);

    //            ed::UUID id = mesh_request_.id;
    //            req.setShape(id, shape);
    //            req.setPose(id, geo::Pose3D(geo::Matrix3::identity(), origin));

    //            if (!mesh_request_.type.empty())
    //                req.setType(id, mesh_request_.type);
    //        }

    //        mesh_request_.id.clear();
    //    }

    // - - - - - - - - - - - - - - - - - -
    // Convex hull association



    // - - - - - - - - - - - - - - - - - -
    // Clear unassociated entities in view

    //    for(unsigned int i = 0; i < entities_associated.size(); ++i)
    //    {
    //        const ed::EntityConstPtr& e = entities[i];

    //        // If the entity is associated, skip it
    //        if (entities_associated[i] >= 0)
    //            continue;

    //        const geo::Pose3D& pose = e->pose();

    //        geo::Vector3 p = sensor_pose.inverse() * pose.t;

    //        cv::Point2d p_2d = cam_model.project3Dto2D(p);

    //        if ( p_2d.x >    border_padding_  * view.getWidth() && p_2d.y >    border_padding_  * view.getHeight() &&
    //             p_2d.x < (1-border_padding_) * view.getWidth() && p_2d.y < (1-border_padding_) * view.getHeight() )
    //        {
    //            float d = depth.at<float>(p_2d);
    //            if (d > 0 && -p.z < d)
    //            {
    ////                std::cout << view.getWidth() << " x " << view.getHeight() << std::endl;
    ////                std::cout << "Should see " << e->id() << ", but I do not (" << p_2d << ")" << std::endl;

    //                double p_exist = e->existenceProbability();
    //                if (p_exist < 0.3) // TODO: magic number
    //                    req.removeEntity(e->id());
    //                else
    //                {
    //                    req.setExistenceProbability(e->id(), std::max(0.0, p_exist - 0.3));  // TODO: very ugly prob update
    //                }
    //            }
    //        }
    //    }

    if (debug_)
        std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Visualize (will only send out images if someone's listening to them)

    //    visualizeNormals(*pc, viz_sensor_normals_);
    //    visualizeNormals(*pc_model, viz_model_normals_);
    //    visualizeClusters(depth, clusters, viz_clusters_);
    visualizeUpdateRequest(world, req, rgbd_image, viz_update_req_);
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvSegment(ed_sensor_integration::Segment::Request& req, ed_sensor_integration::Segment::Response& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Setting parameters

    max_range_ = req.max_sensor_range;

    if (req.enable_continuous_mode)
    {
        continuous_ = true;
        return true;
    }

    if (req.disable_continuous_mode)
    {
        continuous_ = false;
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
    {
        ROS_WARN("[ED ROBOCUP] No RGBD image available");
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    geo::Pose3D sensor_pose;

    if (!tf_listener_->waitForTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(1.0)))
    {
        ROS_WARN("[ED ROBOCUP] Could not get camera pose");
        return true;
    }

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED ROBOCUP] Could not get camera pose: %s", ex.what());
        return true;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<Cluster> clusters;
    segmenter_.segment(*world_, *rgbd_image, sensor_pose, clusters);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    associateAndUpdate(*world_, clusters, rgbd_image, sensor_pose, max_range_, *update_req_);

    for(std::set<ed::UUID>::const_iterator it = update_req_->updated_entities.begin(); it != update_req_->updated_entities.end(); ++it)
    {
        const ed::UUID& id = *it;
        if (update_req_->removed_entities.find(id) == update_req_->removed_entities.end())
            res.entity_ids.push_back(id.str());
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvLockEntities(ed_sensor_integration::LockEntities::Request& req, ed_sensor_integration::LockEntities::Response& res)
{
    //    for(std::vector<std::string>::const_iterator it = req.lock_ids.begin(); it != req.lock_ids.end(); ++it)
    //    {
    //        locked_entities_.insert(*it);
    //        local_ids_.erase(*it);
    //    }

    //    for(std::vector<std::string>::const_iterator it = req.unlock_ids.begin(); it != req.unlock_ids.end(); ++it)
    //        locked_entities_.erase(*it);

    // TODO

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvMeshEntityInView(ed_sensor_integration::MeshEntityInView::Request& req, ed_sensor_integration::MeshEntityInView::Response& res)
{
    mesh_request_ = req;
    res.succeeded = true;
    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

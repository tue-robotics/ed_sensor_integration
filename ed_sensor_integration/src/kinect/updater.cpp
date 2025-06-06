#include "ed/kinect/updater.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <rgbd/view.h>

#include <geolib/Shape.h>
#include <geolib/shapes.h>

#include "ed/kinect/association.h"
#include "ed/kinect/renderer.h"
#include "ed/convex_hull_calc.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ----------------------------------------------------------------------------------------------------

//For displaying SAM MASK
void overlayMasksOnImageOld(cv::Mat& rgb, const std::vector<cv::Mat>& masks)
{
    cv::Mat visualization = rgb.clone();

    // Define a set of distinct colors for different masks
    std::vector<cv::Vec3b> colors = {
        cv::Vec3b(255, 0, 0),     // Blue
        cv::Vec3b(0, 255, 0),     // Green
        cv::Vec3b(0, 0, 255),     // Red
        cv::Vec3b(255, 255, 0),   // Cyan
        cv::Vec3b(255, 0, 255),   // Magenta
        cv::Vec3b(0, 255, 255),   // Yellow
        cv::Vec3b(255, 128, 0),   // Blue-Green
        cv::Vec3b(255, 0, 128)    // Blue-Red
    };

    // Overlay each mask with a different color
    for (size_t i = 0; i < masks.size(); i++) {
        const cv::Mat& mask = masks[i];
        cv::Vec3b color = colors[i % colors.size()];

        // For each pixel in the mask
        for (int y = 0; y < mask.rows; y++) {
            for (int x = 0; x < mask.cols; x++) {
                // If this pixel belongs to the mask
                if (mask.at<uint8_t>(y, x) > 0) {
                    // Blend with original image (50% transparency)
                    cv::Vec3b& rgb_pixel = rgb.at<cv::Vec3b>(y, x);
                    rgb_pixel[0] = (rgb_pixel[0] + color[0]) / 2;
                    rgb_pixel[1] = (rgb_pixel[1] + color[1]) / 2;
                    rgb_pixel[2] = (rgb_pixel[2] + color[2]) / 2;
                }
            }
        }
    }
}
//For displaying SAM MASK
void overlayMasksOnImage(cv::Mat& rgb, const std::vector<cv::Mat>& masks)
{
    // Define colors in BGR format for OpenCV (high contrast)
    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 0, 255),     // Red
        cv::Scalar(0, 255, 0),     // Green
        cv::Scalar(255, 0, 0),     // Blue
        cv::Scalar(0, 255, 255),   // Yellow
        cv::Scalar(255, 0, 255),   // Magenta
        cv::Scalar(255, 255, 0),   // Cyan
        cv::Scalar(128, 0, 128),   // Purple
        cv::Scalar(0, 128, 128)    // Brown
    };

    // Create a copy for the overlay (preserves original for contours)
    cv::Mat overlay = rgb.clone();

    for (size_t i = 0; i < masks.size(); i++) {
        // Get a working copy of the mask
        cv::Mat working_mask = masks[i].clone();

        // Check if mask needs resizing
        if (working_mask.rows != rgb.rows || working_mask.cols != rgb.cols) {
            cv::resize(working_mask, working_mask, rgb.size(), 0, 0, cv::INTER_NEAREST);
        }

        // Ensure the mask is binary (values 0 or 255)
        if (cv::countNonZero((working_mask > 0) & (working_mask < 255)) > 0) {
            cv::threshold(working_mask, working_mask, 127, 255, cv::THRESH_BINARY);
        }

        // Use a different color for each mask
        cv::Scalar color = colors[i % colors.size()];

        // Create the colored overlay with this mask's specific color
        cv::Mat colorMask = cv::Mat::zeros(rgb.size(), CV_8UC3);
        colorMask.setTo(color, working_mask);

        // Add this mask's overlay to the combined overlay
        cv::addWeighted(overlay, 1.0, colorMask, 0.2, 0, overlay);

        // Find contours of the mask - use CHAIN_APPROX_NONE for most accurate contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(working_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // Draw double contours for better visibility (outer black, inner colored)
        cv::drawContours(rgb, contours, -1, cv::Scalar(0, 0, 0), 2);      // Outer black border
        cv::drawContours(rgb, contours, -1, color, 1);                     // Inner colored line

        // // Add mask ID (optional)
        // if (!contours.empty()) {
        //     // Find centroid of largest contour for label placement
        //     int largest_idx = 0;
        //     double largest_area = 0;
        //     for (size_t j = 0; j < contours.size(); j++) {
        //         double area = cv::contourArea(contours[j]);
        //         if (area > largest_area) {
        //             largest_area = area;
        //             largest_idx = j;
        //         }
        //     }

        //     // Calculate centroid
        //     cv::Moments mu = cv::moments(contours[largest_idx]);
        //     if (mu.m00 != 0) {
        //         cv::Point centroid(mu.m10/mu.m00, mu.m01/mu.m00);

        //         // Draw ID with contrasting background
        //         std::string label = std::to_string(i);
        //         int baseline = 0;
        //         cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        //         cv::rectangle(rgb,
        //                       cv::Point(centroid.x - text_size.width/2 - 2, centroid.y - text_size.height/2 - 2),
        //                       cv::Point(centroid.x + text_size.width/2 + 2, centroid.y + text_size.height/2 + 2),
        //                       cv::Scalar(255, 255, 255), -1);

        //         cv::putText(rgb, label,
        //                    cv::Point(centroid.x - text_size.width/2, centroid.y + text_size.height/2),
        //                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        //     }
        // }
    }

    // Apply the semi-transparent overlay with all masks
    cv::addWeighted(rgb, 0.7, overlay, 0.3, 0, rgb);
}

// Calculates which depth points are in the given convex hull (in the EntityUpdate), updates the mask,
// and updates the convex hull height based on the points found
void refitConvexHull(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const geo::DepthCamera& cam_model,
                     const Segmenter& segmenter_, EntityUpdate& up)
{
    up.pose_map.t.z += (up.chull.z_max + up.chull.z_min) / 2;

    geo::Shape chull_shape;
    std::vector<geo::Vec2> points(up.chull.points.size());
    for(unsigned int i = 0; i < points.size(); ++i)
        points[i] = geo::Vec2(up.chull.points[i].x, up.chull.points[i].y);
    geo::createConvexPolygon(chull_shape, points, up.chull.height());

    cv::Mat filtered_depth_image;
    segmenter_.calculatePointsWithin(image, chull_shape, sensor_pose.inverse() * up.pose_map, filtered_depth_image);

    up.points.clear();
    up.pixel_indices.clear();

    float z_min =  1e9;
    float z_max = -1e9;

    int i_pixel = 0;
    for(int y = 0; y < filtered_depth_image.rows; ++y)
    {
        for(int x = 0; x < filtered_depth_image.cols; ++x)
        {
            float d = filtered_depth_image.at<float>(y, x);
            if (d == 0)
            {
                ++i_pixel;
                continue;
            }

            geo::Vec3 p = cam_model.project2Dto3D(x, y) * d;
            geo::Vec3 p_map = sensor_pose * p;

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);

            up.pixel_indices.push_back(i_pixel);
            up.points.push_back(p);
            ++i_pixel;
        }
    }

    double h = z_max - z_min;
    up.pose_map.t.z = (z_max + z_min) / 2;
    up.chull.z_min = -h / 2;
    up.chull.z_max =  h / 2;
}

// ----------------------------------------------------------------------------------------------------
/**
 * @brief mergeConvexHulls, creating a new convexHull around the two objects. Working in both XY and Z.
 * @param u1 EntityUpdate used as starting point
 * @param u2 Merge points into u1
 * @return new EntityUpdate including new convexHull and measurement points of both inputs.
 */
EntityUpdate mergeConvexHulls(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const geo::DepthCamera& cam_model,
                              const Segmenter& segmenter_, const EntityUpdate& u1, const EntityUpdate& u2)
{
    EntityUpdate new_u = u1;
    double z_max = std::max(u1.pose_map.t.getZ()+u1.chull.z_max,u2.pose_map.t.getZ()+u2.chull.z_max);
    double z_min = std::min(u1.pose_map.t.getZ()+u1.chull.z_min,u2.pose_map.t.getZ()+u2.chull.z_min);

    std::vector<geo::Vec2f> points(u1.chull.points.size()+u2.chull.points.size());
    for (unsigned int p = 0; p < u1.chull.points.size(); ++p)
    {
        geo::Vec3 p_map = u1.pose_map * geo::Vec3(u1.chull.points[p].x, u1.chull.points[p].y, 0);
        points[p] = geo::Vec2f(p_map.x, p_map.y);
    }
    unsigned int offset = u1.chull.points.size();
    for (unsigned int p = 0; p < u2.chull.points.size(); ++p)
    {
        geo::Vector3 p_map = u2.pose_map * geo::Vec3(u2.chull.points[p].x, u2.chull.points[p].y, 0);
        points[p + offset] = geo::Vec2f(p_map.x, p_map.y);
    }

    ed::convex_hull::create(points, z_min, z_max, new_u.chull, new_u.pose_map);
    refitConvexHull(image, sensor_pose, cam_model, segmenter_, new_u);

    return new_u;
}

// ----------------------------------------------------------------------------------------------------

// Calculates which depth points are in the given convex hull (in the EntityUpdate), updates the mask,
// and updates the convex hull height based on the points found
std::vector<EntityUpdate> mergeOverlappingConvexHulls(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const geo::DepthCamera& cam_model,
                                                         const Segmenter& segmenter_, const std::vector<EntityUpdate>& updates)
{

  ROS_INFO("mergoverlapping chulls: nr of updates: %lu", updates.size());

  // Updated convex hulls
  std::vector<EntityUpdate> new_updates;

  // Keep track of indices that did collide, skip them in i
  std::vector<int> collided_indices;
  std::map<int, std::vector<int> > collission_map;

  for (uint i = 0; i < updates.size(); ++i)
  {
    const EntityUpdate& u1 = updates[i];

    // If index already collided, it will be merged to another one
    if (std::find(collided_indices.begin(), collided_indices.end(), i) != collided_indices.end())
    {
      continue;
    }

    for (uint j = 0; j < updates.size(); ++j)
    {
    // skip self
        if (i == j)
            continue;

      const EntityUpdate& u2 = updates[j];

      // If we collide, update the i convex hull
      if (ed::convex_hull::collide(u1.chull, u1.pose_map.t, u2.chull, u2.pose_map.t, 0, 1e6))  // This should prevent multiple entities above each other;1e6 is ok, because objects in other areas are ignored.
      {
        ROS_DEBUG("Collition item %i with %i", i, j);
        ROS_DEBUG("Item %i: xyz: %.2f, %.2f, %.2f, z_min: %.2f, z_max: %.2f", i, u1.pose_map.t.getX(), u1.pose_map.t.getY(), u1.pose_map.t.getZ(), u1.chull.z_min, u1.chull.z_max);
        ROS_DEBUG("Item %i: xyz: %.2f, %.2f, %.2f, z_min: %.2f, z_max: %.2f", j, u2.pose_map.t.getX(), u2.pose_map.t.getY(), u2.pose_map.t.getZ(), u2.chull.z_min, u2.chull.z_max);
        collission_map[i].push_back(j);
        collided_indices.push_back(j);
      }
    }
  }

  // Now again loop over the updates and only push back in the new updates if it will not be merged into an other entity
  for (uint i = 0; i < updates.size(); ++i)
  {
    // If index in collided_indices, it will be merged to another one
    if (std::find(collided_indices.begin(), collided_indices.end(), i) == collided_indices.end())
    {
      // Merging is done by creating a new convexHull. Multiple objects can be merged into one.
      // No sorting is done. So depending on which entity was taken first, that one is used as basis for merging.
      // But that shouldn't matter, only for UUID. Although the entities are re-associatied afterwards.
      // Which match new measurments to old entities.

      EntityUpdate u1 = updates[i];
      for (std::vector<int>::iterator it = collission_map[i].begin(); it != collission_map[i].end(); ++it)
      {
          ROS_DEBUG_COND(it == collission_map[i].begin(), "Merging entity %i and xx", i);
          ROS_DEBUG("Merging entity %i and %i", i, *it);
          const EntityUpdate u2 = updates[*it];
          u1 = mergeConvexHulls(image, sensor_pose, cam_model, segmenter_, u1, u2);

      }
      new_updates.push_back(u1);
      ROS_DEBUG("Adding update of entity %i", i);
    }
    else
    {
        ROS_DEBUG("Skipping update %i because of collision", i);
    }
  }

  return new_updates;
}

// ----------------------------------------------------------------------------------------------------

Updater::Updater()
{
    // Initialize the image publisher
    ros::NodeHandle nh("~");
    mask_pub_ = nh.advertise<sensor_msgs::Image>("segmentation_masks", 1);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_ooo", 1);
}

// ----------------------------------------------------------------------------------------------------

Updater::~Updater()
{
}

// ----------------------------------------------------------------------------------------------------

bool Updater::update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose_const,
                     const UpdateRequest& req, UpdateResult& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Prepare some things

    // will contain depth image filtered with given update shape and world model (background) subtraction
    cv::Mat filtered_depth_image;

    // sensor pose might be update, so copy (making non-const)
    geo::Pose3D sensor_pose = sensor_pose_const;

    // depth image
    const cv::Mat& depth = image->getDepthImage();

    // Determine depth image camera model
    rgbd::View view(*image, depth.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    std::string area_description;

    if (!req.area_description.empty())
    {
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if the update_command is a segmented entity.
        // If so, lookup the corresponding area_description

        bool fit_supporting_entity = req.fit_supporting_entity;

        std::map<ed::UUID, std::string>::const_iterator it_area_descr = id_to_area_description_.find(req.area_description);
        if (it_area_descr != id_to_area_description_.end())
        {
            area_description = it_area_descr->second;
            fit_supporting_entity = false; // We are only interested in the supported entity, so don't fit the supporting entity
        }
        else
            area_description = req.area_description;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Parse space description (split on space)

        std::size_t i_space = area_description.find(' ');

        ed::UUID entity_id;
        std::string area_name;

        if (i_space == std::string::npos)
        {
            entity_id = area_description;
        }
        else
        {
            area_name = area_description.substr(0, i_space);
            entity_id = area_description.substr(i_space + 1);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check for entity

        ed::EntityConstPtr e = world.getEntity(entity_id);

        if (!e)
        {
            res.error << "No such entity: '" << entity_id.str() << "'.";
            return false;
        }
        else if (!e->has_pose())
        {
            res.error << "Entity: '" << entity_id.str() << "' has no pose.";
            return false;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Update entity position

        geo::Pose3D new_pose;

        if (fit_supporting_entity)
        {
            if (!fitter_.isConfigured())
            {
                fitter_.configureBeamModel(image->getCameraModel());
            }
            FitterData fitter_data;
            fitter_.processSensorData(*image, sensor_pose, fitter_data);

            if (fitter_.estimateEntityPose(fitter_data, world, entity_id, e->pose(), new_pose, req.max_yaw_change))
            {
                res.update_req.setPose(entity_id, new_pose);
            }
            else
            {
//                res.error << "Could not determine pose of '" << entity_id.str() << "'.";
//                return false;

                // Could not fit entity, so keep the old pose
                new_pose = e->pose();
            }
        }
        else
        {
            new_pose = e->pose();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Optimize sensor pose

        if (false)
        {
            fitZRP(*e->visual(), new_pose, *image, sensor_pose_const, sensor_pose);

            ROS_DEBUG_STREAM("Old sensor pose: " << sensor_pose_const);
            ROS_DEBUG_STREAM("New sensor pose: " << sensor_pose);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Determine segmentation area

        if (!area_name.empty())
        {
            // Determine segmentation area (the geometrical shape in which the segmentation should take place)

            std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e->volumes().find(area_name);
            if (it == e->volumes().end())
            {
                res.error << "No area '" << area_name << "' for entity '" << entity_id.str() << "'.";
                return false;
            }
            geo::Shape shape = *(it->second);
            if (shape.getMesh().empty())
            {
                // Empty shapes shouldn't be stored at all, but check for robustness
                res.error << "Could not load shape of area '" << area_name << "' for entity '" << entity_id.str() << "'.";
                return false;
            }

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Segment

            geo::Pose3D shape_pose = sensor_pose.inverse() * new_pose;
            segmenter_.calculatePointsWithin(*image, shape, shape_pose, filtered_depth_image);
        }
    }
    else
    {
        filtered_depth_image = image->getDepthImage().clone();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Remove background

    // The world model may have been updated above, but the changes are only captured in
    // an update request. Therefore, make a (shallow) copy of the world model and apply
    // the changes, and use this for the background removal

    ed::WorldModel world_updated = world;
    world_updated.update(res.update_req);

    segmenter_.removeBackground(filtered_depth_image, world_updated, cam_model, sensor_pose, req.background_padding);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear convex hulls that are no longer there

    std::vector<ed::EntityConstPtr> associatable_entities;
    for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
    {
        const ed::EntityConstPtr& e = *e_it;
        if (e->visual() || !e->has_pose() || e->convexHull().points.empty())
            continue;

        associatable_entities.push_back(e);
        /*
        geo::Vec3 p_3d = sensor_pose.inverse() * e->pose().t;

        cv::Point p_2d = cam_model.project3Dto2D(p_3d);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth.cols || p_2d.y >= depth.rows)
            continue;
        */
        /*float d = depth.at<float>(p_2d);
        if (d > 0 && d == d && -p_3d.z < d)
        {
            ROS_INFO("Request to remove entity %s", e->id().c_str());
            res.update_req.removeEntity(e->id());
            associatable_entities.pop_back();
        }*/
    }

//    cv::imshow("depth image", depth / 10);
//    cv::imshow("segments", filtered_depth_image / 10);
//    cv::waitKey();

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Cluster
    std::vector<cv::Mat> clustered_images = segmenter_.cluster(filtered_depth_image, cam_model, sensor_pose, res.entity_updates);

    // // Overlay masks on the RGB image
    const cv::Mat& rgb = image->getRGBImage();
    cv::Mat visualization = rgb.clone();
    cv::imwrite("/tmp/visualization.png", visualization);

    overlayMasksOnImage(visualization, clustered_images);
    // save after overlaying masks
    cv::imwrite("/tmp/visualization_with_masks.png", visualization);
    // // Convert to ROS message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visualization).toImageMsg();
    msg->header.stamp = ros::Time::now();

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    PointCloud::Ptr combined_cloud (new PointCloud);

    combined_cloud->header.frame_id = "map"; // Use appropriate frame ID

    // Add points from all entity updates
    for (const EntityUpdate& update : res.entity_updates) {
        for (const geo::Vec3& point : update.points) {
            // Transform from camera to map frame
            geo::Vec3 p_map = sensor_pose * point;
            pcl::PointXYZ pcl_point;
            pcl_point.x = p_map.x;
            pcl_point.y = p_map.y;
            pcl_point.z = p_map.z;
            combined_cloud->push_back(pcl_point);
        }
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*combined_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map"; // Use appropriate frame ID

    // // Publish
    mask_pub_.publish(msg);
    cloud_pub_.publish(cloud_msg);
    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Merge the detected clusters if they overlap in XY or Z
    res.entity_updates = mergeOverlappingConvexHulls(*image, sensor_pose, cam_model, segmenter_, res.entity_updates);

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Increase the convex hulls a bit towards the supporting surface and re-calculate mask
    // Then shrink the convex hulls again to get rid of the surface pixels

    for(std::vector<EntityUpdate>::iterator it = res.entity_updates.begin(); it != res.entity_updates.end(); ++it)
    {
        EntityUpdate& up = *it;

        up.chull.z_min -= 0.04;
        refitConvexHull(*image, sensor_pose, cam_model, segmenter_, up);

        up.chull.z_min += 0.01;
        refitConvexHull(*image, sensor_pose, cam_model, segmenter_, up);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Perform association and update
    associateAndUpdate(associatable_entities, image, sensor_pose, res.entity_updates, res.update_req);

    // - - - - - - - - - - - - -  - - - - - - - -  - - -
    // Remove entities that are not associated
    for (std::vector<ed::EntityConstPtr>::const_iterator it = associatable_entities.begin(); it != associatable_entities.end(); ++it)
    {
        ed::EntityConstPtr e = *it;

        // Check if entity is in frustum
        const geo::Vec3 p_3d = sensor_pose.inverse() * e->pose().t; // Only taking into account the pose, not the shape
        const cv::Point p_2d = cam_model.project3Dto2D(p_3d);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth.cols || p_2d.y >= depth.rows)
            continue; // Outside of frustum

        // If the entity is not updated, remove it
        if (res.update_req.updated_entities.find(e->id()) == res.update_req.updated_entities.end())
        {
            ROS_INFO("Entity not associated and not seen in the frustum, while seeable");

            float d = depth.at<float>(p_2d);
            if (d > 0 && d == d && -p_3d.z < d)
            {
                ROS_INFO_STREAM("We can shoot a ray through the center(" << d << " > " << -p_3d.z << "), removing entity " << e->id());
                res.update_req.removeEntity(e->id());
                res.removed_entity_ids.push_back(e->id());
            }
        }

    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Remember the area description with which the segments where found

    if (!area_description.empty())
    {
       for(std::vector<EntityUpdate>::const_iterator it = res.entity_updates.begin(); it != res.entity_updates.end(); ++it)
        {
            const EntityUpdate& up = *it;
            id_to_area_description_[up.id] = area_description;
        }
    }

    return true;
}

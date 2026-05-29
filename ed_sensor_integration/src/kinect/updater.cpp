#include "ed/kinect/updater.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/update_request.h>

#include <geolib/Shape.h>

#include "ed/kinect/association.h"
#include "ed/kinect/renderer.h"
#include "ed/convex_hull_calc.h"

#include <opencv2/highgui/highgui.hpp>
#include <rgbd/view.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "ed_sensor_integration/kinect/segmodules/sam_seg_module.h"
// ----------------------------------------------------------------------------------------------------

Updater::Updater(tue::Configuration config) : verbose(false)
{
    if (config.readGroup("segmenter", tue::config::REQUIRED))
    {
        segmenter_ = std::make_unique<Segmenter>(config);
    }
    else
    {
        ROS_ERROR("Failed to read segmenter configuration group from config file, cannot initialize Updater");
        throw std::runtime_error("Failed to read segmenter configuration group from config file");
    }

    // Initialize the image publisher
    config.value("verbose", verbose, tue::config::OPTIONAL);
    if (verbose)
    {
        ros::NodeHandle nh("~");
        mask_pub_ = nh.advertise<sensor_msgs::Image>("segmentation_masks", 1);
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_sam", 1);
        box_pub_ = nh.advertise<sensor_msgs::Image>("bounding_boxes_yolo", 1);
    }
}

// ----------------------------------------------------------------------------------------------------

Updater::~Updater()
{
}

// ----------------------------------------------------------------------------------------------------

bool Updater::update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose_const,
                     const UpdateRequest& req, UpdateResult& res)
{
    ed::ErrorContext errc("Kinect::Updater", "update");
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Prepare some things

    // will contain depth image filtered with given update shape and world model (background) subtraction
    cv::Mat filtered_depth_image;

    // sensor pose might be update, so copy (making non-const)
    geo::Pose3D sensor_pose = sensor_pose_const;

    // depth image
    const cv::Mat& depth_image = image->getDepthImage();
    const cv::Mat& rgb_image = image->getRGBImage();

    // Determine depth image camera model
    rgbd::View view(*image, depth_image.cols);
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
            //  res.error << "Could not determine pose of '" << entity_id.str() << "'.";
            //  return false;

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
            segmenter_->calculatePointsWithin(*image, shape, shape_pose, filtered_depth_image);
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

    errc.change("Kinect::Updater", "update: preparing world model");


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
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth_image.cols || p_2d.y >= depth_image.rows)
            continue;
        */
        /*float d = depth_image.at<float>(p_2d);
        if (d > 0 && d == d && -p_3d.z < d)
        {
            ROS_INFO("Request to remove entity %s", e->id().c_str());
            res.update_req.removeEntity(e->id());
            associatable_entities.pop_back();
        }*/
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Cluster
    errc.change("Kinect::Updater", "update: clustering");

    // Do preprocessRGBForSegmentation if you want to treat the rgb image as the depth image for segmentation, meaning that only the pixels where depth is non-zero will be kept in the RGB image. This can be used to improve the segmentation results of the RGB image, but it is not necessary for the depth segmentation.
    // cv::Mat filtered_rgb_image;
    // filtered_rgb_image = segmenter_->preprocessRGBForSegmentation(rgb_image, filtered_depth_image);
    SegmentationResult cluster_result = segmenter_->cluster(filtered_depth_image, cam_model, sensor_pose, res.entity_updates, rgb_image, area_description, verbose);

    std::vector<cv::Mat>& clustered_images = cluster_result.masks;

    if (verbose)
    {
        publishSegmentationResults(filtered_depth_image, rgb_image, sensor_pose, clustered_images, cluster_result.boxes, res.entity_updates);
    }
    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Perform association and update
    errc.change("Kinect::Updater", "update: association");
    associateAndUpdate(associatable_entities, image, sensor_pose, res.entity_updates, res.update_req);

    // - - - - - - - - - - - - -  - - - - - - - -  - - -
    // Remove entities that are not associated
    errc.change("Kinect::Updater", "update: remove unassociated entities");
    for (std::vector<ed::EntityConstPtr>::const_iterator it = associatable_entities.begin(); it != associatable_entities.end(); ++it)
    {
        ed::EntityConstPtr e = *it;

        // Check if entity is in frustum
        const geo::Vec3 p_3d = sensor_pose.inverse() * e->pose().t; // Only taking into account the pose, not the shape
        const cv::Point p_2d = cam_model.project3Dto2D(p_3d);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth_image.cols || p_2d.y >= depth_image.rows)
            continue; // Outside of frustum

        // If the entity is not updated, remove it
        if (res.update_req.updated_entities.find(e->id()) == res.update_req.updated_entities.end())
        {
            ROS_INFO("Entity not associated and not seen in the frustum, while seeable");

            float d = depth_image.at<float>(p_2d);
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
    errc.change("Kinect::Updater", "update: store area description to segments");
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

void Updater::publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                const std::vector<cv::Rect>& boxes, std::vector<EntityUpdate>& res_updates)
{
    // Guard: all three publishers must have been advertised before calling this function.
    // They are only advertised when verbose == true in the Updater constructor.
    // If they evaluate to false (meaning they are null/unadvertised), abort publishing.
    if (!this->box_pub_ || !this->mask_pub_ || !this->cloud_pub_)
    {
        ROS_ERROR_THROTTLE(1.0, "publishSegmentationResults called but publishers are not active. Did you set verbose: true?");
        return;
    }

    // Overlay masks on the RGB image
    cv::Mat visualization = rgb.clone();
    cv::Mat box_visualization = rgb.clone();
    for(const auto& box : boxes)
    {
        cv::rectangle(box_visualization, box, cv::Scalar(0, 255, 0), 2);
    }

    sensor_msgs::ImagePtr box_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", box_visualization).toImageMsg();
    box_msg->header.stamp = ros::Time::now();
    this->box_pub_.publish(box_msg);

    // Create a path to save the image using platform-independent temp directory
    std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
    cv::imwrite((temp_dir / "visualization.png").string(), visualization);

    // Create a properly normalized depth visualization
    cv::Mat depth_vis;
    double min_val, max_val;
    cv::minMaxLoc(filtered_depth_image, &min_val, &max_val);

    // Handle empty depth image case
    if (max_val == 0)
    {
        depth_vis = cv::Mat::zeros(filtered_depth_image.size(), CV_8UC1);
    }
    else
    {
        // Scale to full 8-bit range and convert to 8-bit
        filtered_depth_image.convertTo(depth_vis, CV_8UC1, 255.0 / max_val);

        // Apply a colormap for better visibility
        cv::Mat depth_color;
        cv::applyColorMap(depth_vis, depth_color, cv::COLORMAP_JET);
        cv::imwrite((temp_dir / "visualization_depth_color.png").string(), depth_color);
    }

    // Save both grayscale and color versions
    cv::imwrite((temp_dir / "visualization_depth.png").string(), depth_vis);
    overlayMasksOnImage_(visualization, clustered_images);
    // save after overlaying masks
    cv::imwrite((temp_dir / "visualization_with_masks.png").string(), visualization);

    // Convert to ROS message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visualization).toImageMsg();
    msg->header.stamp = ros::Time::now();

    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    PointCloud::Ptr combined_cloud (new PointCloud);

    combined_cloud->header.frame_id = "map";

    // Add points from all entity updates
    for (const EntityUpdate& update : res_updates)
    {
        for (const geo::Vec3& point : update.points)
        {
            // Transform from camera to map frame
            geo::Vec3 p_map = sensor_pose * point;
            pcl::PointXYZRGB pcl_point;
            pcl_point.x = p_map.x;
            pcl_point.y = p_map.y;
            pcl_point.z = p_map.z;
            pcl_point.r = 255;  // White
            pcl_point.g = 255;
            pcl_point.b = 255;
            combined_cloud->push_back(pcl_point);
        }

        // Add outlier points (red)
        for (const geo::Vec3& point : update.outlier_points) {
                geo::Vec3 p_map = sensor_pose * point;
                pcl::PointXYZRGB pcl_point;
                pcl_point.x = p_map.x;
                pcl_point.y = p_map.y;
                pcl_point.z = p_map.z;
                pcl_point.r = 255;  // Red
                pcl_point.g = 0;
                pcl_point.b = 0;
                combined_cloud->push_back(pcl_point);
        }
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*combined_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map"; // Use appropriate frame ID

    // Publish
    this->mask_pub_.publish(msg);
    this->cloud_pub_.publish(cloud_msg);
}

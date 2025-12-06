#include "ed_sensor_integration/kinect/segmodules/sam_seg_module.h"
#include "sam_onnx_ros/segmentation.hpp"
#include "yolo_onnx_ros/detection.hpp"
#include <algorithm>
// Add required includes for types used in publishSegmentationResults
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::vector<cv::Mat> SegmentationPipeline(const cv::Mat& img, tue::Configuration& config)
{

    ////////////////////////// YOLO //////////////////////////////////////
    std::unique_ptr<YOLO_V8> yoloDetector;
    DL_INIT_PARAM params;
    std::string yolo_model, sam_encoder, sam_decoder;
    config.value("yolo_model", yolo_model);
    config.value("sam_encoder", sam_encoder);
    config.value("sam_decoder", sam_decoder);

    std::tie(yoloDetector, params) = Initialize(yolo_model);
    ////////////////////////// SAM //////////////////////////////////////

    std::vector<std::unique_ptr<SAM>> samSegmentors;
    SEG::DL_INIT_PARAM params_encoder;
    SEG::DL_INIT_PARAM params_decoder;
    std::vector<SEG::DL_RESULT> resSam;
    SEG::DL_RESULT res;
    std::tie(samSegmentors, params_encoder, params_decoder, res, resSam) = Initialize(sam_encoder, sam_decoder);


    ////////////////////////// YOLO //////////////////////////////////////
    std::vector<DL_RESULT> resYolo;
    resYolo = Detector(yoloDetector, img);

    ////////////////////////// SAM //////////////////////////////////////
    for (const auto& result : resYolo) {
        // This should be deleted as we would upload everything
        // (here we are skipping the table object but it should happen only on the rosservice scenario: on_top_of dinner_table )
        int table_classification = 60;
        if (result.classId == table_classification) {
            std::cout << "Class ID is: " << yoloDetector->classes[result.classId] << " So we dont append"<< std::endl;
            continue;
        }
        res.boxes.push_back(result.box);
        std::cout << "Confidence is OKOK: " << result.confidence << std::endl;
        std::cout << "Class is: " << yoloDetector->classes[result.classId] << std::endl;
        std::cout << "Class ID is: " << result.classId << std::endl;
    }

    SegmentAnything(samSegmentors, params_encoder, params_decoder, img, resSam, res);

    return std::move(res.masks);
    }


void overlayMasksOnImage_(cv::Mat& rgb, const std::vector<cv::Mat>& masks)
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
        if (cv::countNonZero(working_mask > 0 & working_mask < 255) > 0) {
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

    }

    // Apply the semi-transparent overlay with all masks
    cv::addWeighted(rgb, 0.7, overlay, 0.3, 0, rgb);
}

void publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                ros::Publisher& mask_pub_, ros::Publisher& cloud_pub_, std::vector<EntityUpdate>& res_updates)
{
    // Overlay masks on the RGB image
    cv::Mat visualization = rgb.clone();
    // Create a path to save the image
    std::string path = "/tmp";
    cv::imwrite(path + "/visualization.png", visualization);

    // Create a properly normalized depth visualization
    cv::Mat depth_vis;
    double min_val, max_val;
    cv::minMaxLoc(filtered_depth_image, &min_val, &max_val);

    // Handle empty depth image case
    if (max_val == 0) {
        depth_vis = cv::Mat::zeros(filtered_depth_image.size(), CV_8UC1);
    } else {
        // Scale to full 8-bit range and convert to 8-bit
        filtered_depth_image.convertTo(depth_vis, CV_8UC1, 255.0 / max_val);

        // Apply a colormap for better visibility
        cv::Mat depth_color;
        cv::applyColorMap(depth_vis, depth_color, cv::COLORMAP_JET);
        cv::imwrite(path + "/visualization_depth_color.png", depth_color);
    }

    // Save both grayscale and color versions
    cv::imwrite(path + "/visualization_depth.png", depth_vis);
    overlayMasksOnImage_(visualization, clustered_images);
    // save after overlaying masks
    cv::imwrite(path + "/visualization_with_masks.png", visualization);
    // Convert to ROS message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visualization).toImageMsg();
    msg->header.stamp = ros::Time::now();

    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    PointCloud::Ptr combined_cloud (new PointCloud);

    combined_cloud->header.frame_id = "map";

    // Add inlier points (white)
    for (const EntityUpdate& update : res_updates) {
        for (const geo::Vec3& point : update.points) {
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
    mask_pub_.publish(msg);
    cloud_pub_.publish(cloud_msg);
}

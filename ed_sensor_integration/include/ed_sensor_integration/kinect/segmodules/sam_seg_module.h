#pragma once

#include <ros/ros.h>
#include "yolo_onnx_ros/yolo_inference.hpp"
#include "sam_onnx_ros/sam_inference.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
//#include "ed/kinect/entity_update.h"
#include <cv_bridge/cv_bridge.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tue/config/configuration.h>

// EntityUpdate and UpdateResult come from kinect (avoid including laser variant to prevent redefinition)
#include <ed/kinect/entity_update.h>
#include <ed/kinect/segmenter.h>  // defines UpdateResult (and possibly other needed types)

/**
 * @brief Segmentation pipeline that processes the input image and generates segmentation masks.
 *
 * @param img The input RGB image to segment.
 * @return std::vector<cv::Mat> The generated segmentation masks.
 */
std::vector<cv::Mat> SegmentationPipeline(const cv::Mat& img, tue::Configuration& config);

/**
 * @brief Overlay segmentation masks on the RGB image for visualization purposes.
 *
 * @param rgb The RGB image to overlay masks on.
 * @param masks The segmentation masks to overlay.
 */
void overlayMasksOnImage_(cv::Mat& rgb, const std::vector<cv::Mat>& masks);

/**
 * @brief Publish segmentation results and pointcloud estimation as ROS messages.
 *
 * @param filtered_depth_image The filtered depth image to publish.
 * @param rgb The RGB image to publish.
 * @param sensor_pose The pose of the sensor.
 * @param clustered_images The clustered segmentation masks.
 * @param mask_pub_ The ROS publisher for the mask images.
 * @param cloud_pub_ The ROS publisher for the point cloud data.
 * @param res_updates The entity updates to publish.
 */
void publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                ros::Publisher& mask_pub_, ros::Publisher& cloud_pub_, std::vector<EntityUpdate>& res_updates);

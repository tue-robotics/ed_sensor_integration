#pragma once

#include <ros/ros.h>
#include "yolo_inference.h"
#include "sam_inference.h"
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

// EntityUpdate and UpdateResult come from kinect (avoid including laser variant to prevent redefinition)
#include <ed/kinect/entity_update.h>
#include <ed/kinect/segmenter.h>  // defines UpdateResult (and possibly other needed types)

std::vector<cv::Mat> SegmentationPipeline(const cv::Mat& img);
void overlayMasksOnImage_(cv::Mat& rgb, const std::vector<cv::Mat>& masks);
void publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                ros::Publisher& mask_pub_, ros::Publisher& cloud_pub_, std::vector<EntityUpdate>& res_updates);

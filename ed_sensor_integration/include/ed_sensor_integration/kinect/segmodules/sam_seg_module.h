#pragma once

#include <opencv2/core.hpp>

#include <geolib/datatypes.h>
#include <ros/publisher.h>
#include <tue/config/configuration.h>

#include "ed/kinect/entity_update.h"

#include <string>
#include <utility>
#include <vector>

/**
 * @brief Latency measurements for the segmentation pipeline subsystems.
 *        All times are in milliseconds.
 */
struct SegmentationLatency
{
    double yolo_init_ms = 0.0;    // YOLO model initialization time
    double yolo_infer_ms = 0.0;   // YOLO inference time
    double sam_init_ms = 0.0;     // SAM model initialization time
    double sam_infer_ms = 0.0;    // SAM inference time
    double total_pipeline_ms = 0.0; // Total pipeline time
};

/**
 * @brief Result of the segmentation pipeline containing masks, bounding boxes,
 *        and YOLO classification info for each detected object.
 */
struct SegmentationResult
{
    std::vector<cv::Mat> masks;
    std::vector<cv::Rect> boxes;
    std::vector<std::string> labels;      // YOLO class name per detection
    std::vector<float> confidences;       // YOLO confidence per detection
    SegmentationLatency latency;          // Timing measurements (populated when verbose=true)
};

/**
 * @brief Segmentation pipeline class that maintains YOLO and SAM models state.
 *        This prevents initializing heavy models iteratively on every frame.
 */
class SamSegPipeline
{
public:
    SamSegPipeline();
    ~SamSegPipeline();

    /**
     * @brief Initialize the YOLO and SAM models from configuration.
     * @param config Configuration containing model paths.
     */
    void initialize(tue::Configuration& config);

    /**
     * @brief Process the input image and generate segmentation masks.
     * @param img The input RGB image to segment.
     * @param depth_image Optionally, the filtered depth image to ignore bounding boxes outside our point cloud frustum.
     * @param ignore_label Optionally, a specific YOLO label to skip inferring SAM on.
     * @param verbose If true, measure and populate latency timings.
     * @return SegmentationResult The generated masks, bounding boxes, labels, and confidences.
     */
    SegmentationResult process(const cv::Mat& img, const cv::Mat& depth_image = cv::Mat(), const std::string& ignore_label = "", bool verbose = false);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    bool is_initialized_;
};

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
 * @param boxes The bounding boxes to visualize.
 * @param mask_pub_ The ROS publisher for the mask images.
 * @param cloud_pub_ The ROS publisher for the point cloud data.
 * @param res_updates The entity updates to publish.
 */
void publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                const std::vector<cv::Rect>& boxes,
                                ros::Publisher& box_pub_, ros::Publisher& mask_pub_, ros::Publisher& cloud_pub_, std::vector<EntityUpdate>& res_updates);

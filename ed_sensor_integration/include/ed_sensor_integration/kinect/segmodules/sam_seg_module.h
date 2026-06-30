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
 * @brief Result of the segmentation pipeline containing masks, bounding boxes,
 *        and YOLO classification info for each detected object.
 */
struct SegmentationResult
{
    std::vector<cv::Mat> masks;
    std::vector<cv::Rect> boxes;
    std::vector<std::string> labels;      // YOLO class name per detection
    std::vector<float> confidences;       // YOLO confidence per detection
};

/**
 * @brief Segmentation pipeline that processes the input image and generates segmentation masks.
 *
 * @param img The input RGB image to segment.
 * @param config Configuration containing model paths.
 * @return SegmentationResult The generated segmentation masks, bounding boxes, labels, and confidences.
 */
SegmentationResult SegmentationPipeline(const cv::Mat& img, tue::Configuration& config);

/**
 * @brief Overlay segmentation masks on the RGB image for visualization purposes.
 *
 * @param rgb The RGB image to overlay masks on.
 * @param masks The segmentation masks to overlay.
 */
void overlayMasksOnImage_(cv::Mat& rgb, const std::vector<cv::Mat>& masks);

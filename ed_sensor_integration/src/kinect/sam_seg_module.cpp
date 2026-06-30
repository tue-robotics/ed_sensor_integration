#include "ed_sensor_integration/kinect/segmodules/sam_seg_module.h"

#include <filesystem>
#include <utility>
#include <sam_onnx_ros/segmentation.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolo_onnx_ros/detection.hpp>


SegmentationResult SegmentationPipeline(const cv::Mat& img, tue::Configuration& config)
{
    SegmentationResult seg_result;

    ////////////////////////// YOLO //////////////////////////////////////
    std::unique_ptr<YOLO_V8> yoloDetector;
    DL_INIT_PARAM params;
    std::string yolo_model, sam_encoder, sam_decoder;
    config.value("yolo_model", yolo_model);
    std::tie(yoloDetector, params) = Initialize(yolo_model);

    ////////////////////////// SAM //////////////////////////////////////
    std::vector<std::unique_ptr<SAM>> samSegmentors;
    SEG::DL_INIT_PARAM params_encoder;
    SEG::DL_INIT_PARAM params_decoder;
    std::vector<SEG::DL_RESULT> resSam;
    SEG::DL_RESULT res;
    config.value("sam_encoder", sam_encoder);
    config.value("sam_decoder", sam_decoder);
    std::tie(samSegmentors, params_encoder, params_decoder, res, resSam) = Initialize(sam_encoder, sam_decoder);

    ////////////////////////// YOLO //////////////////////////////////////
    std::vector<DL_RESULT> resYolo;
    resYolo = Detector(yoloDetector, img);

    ////////////////////////// SAM //////////////////////////////////////
    for (const auto& result : resYolo)
    {
        res.boxes.push_back(result.box);
        seg_result.labels.push_back(yoloDetector->classes[result.classId]);
        seg_result.confidences.push_back(result.confidence);
        ROS_DEBUG("Confidence: %f", result.confidence);
        ROS_DEBUG("Class: %s", yoloDetector->classes[result.classId].c_str());
        ROS_DEBUG("Class ID: %d", result.classId);
    }

    SegmentAnything(samSegmentors, params_encoder, params_decoder, img, resSam, res);

    seg_result.masks = std::move(res.masks);
    seg_result.boxes = std::move(res.boxes);

    // Verify 1:1 alignment between masks and labels.
    // Normally guaranteed because SAM processes boxes sequentially and pushes one mask per box.
    // If SAM failed for any box it pushes an empty placeholder (see sam_onnx_ros/src/utils.cpp),
    // so sizes should always match. If they don't, we cannot know which label belongs to which
    // mask — discard labels for this frame so objects are still detected but not mislabeled.
    if (seg_result.masks.size() != seg_result.labels.size())
    {
        ROS_WARN("SAM produced %zu masks for %zu YOLO boxes — alignment broken, discarding labels this frame",
                 seg_result.masks.size(), seg_result.labels.size());
        seg_result.labels.clear();
        seg_result.confidences.clear();
    }

    return seg_result;
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

    for (size_t i = 0; i < masks.size(); i++)
    {
        // Get a working copy of the mask
        cv::Mat working_mask = masks[i].clone();

        // Check if mask needs resizing
        if (working_mask.rows != rgb.rows || working_mask.cols != rgb.cols)
            cv::resize(working_mask, working_mask, rgb.size(), 0, 0, cv::INTER_NEAREST);

        // Ensure the mask is binary (values 0 or 255)
        if (cv::countNonZero((working_mask > 0) & (working_mask < 255)) > 0)
            cv::threshold(working_mask, working_mask, 127, 255, cv::THRESH_BINARY);

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
        cv::drawContours(rgb, contours, -1, cv::Scalar(0, 0, 0), 2); // Outer black border
        cv::drawContours(rgb, contours, -1, color, 1); // Inner colored line
    }

    // Apply the semi-transparent overlay with all masks
    cv::addWeighted(rgb, 0.7, overlay, 0.3, 0, rgb);
}

#include "ed_sensor_integration/kinect/segmodules/sam_seg_module.h"

#include <filesystem>
#include <utility>
#include <sam_onnx_ros/segmentation.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolo_onnx_ros/detection.hpp>
#include <chrono>
#include <cstdio>



struct SamSegPipeline::Impl
{
    std::unique_ptr<YOLO_V8> yoloDetector;
    DL_INIT_PARAM yolo_params;

    SamWrapper samWrapper;
    SEG::DL_INIT_PARAM sam_encoder_params;
    SEG::DL_INIT_PARAM sam_decoder_params;
    SEG::DL_RESULT sam_res;
    std::vector<SEG::DL_RESULT> resSam;
};

SamSegPipeline::SamSegPipeline() : pimpl_(std::make_unique<Impl>()), is_initialized_(false) {}

SamSegPipeline::~SamSegPipeline() = default;

void SamSegPipeline::initialize(tue::Configuration& config)
{
    if (is_initialized_)
        return;

    ////////////////////////// YOLO INITIALIZATION //////////////////////////////////////
    std::string yolo_model, sam_encoder, sam_decoder;
    config.value("yolo_model", yolo_model);
    std::tie(pimpl_->yoloDetector, pimpl_->yolo_params) = Initialize(yolo_model);

    ////////////////////////// SAM INITIALIZATION //////////////////////////////////////
    config.value("sam_encoder", sam_encoder);
    config.value("sam_decoder", sam_decoder);

    std::string backend_str;
    SEG::Backend backend = SEG::Backend::kOnnx;
    if (config.value("sam_backend", backend_str))
    {
        if (backend_str == "tensorRT")
        {
            backend = SEG::Backend::kSpeedSam;
            ROS_WARN(" LETS USE SPEEDSAM! ");
        }
        else if (backend_str == "onnx")
        {
            backend = SEG::Backend::kOnnx;
        }
        else
        {
            ROS_WARN("Unsupported SAM backend '%s'. Defaulting to ONNX.", backend_str.c_str());
        }
    }

    std::tie(pimpl_->samWrapper, pimpl_->sam_encoder_params, pimpl_->sam_decoder_params, pimpl_->sam_res, pimpl_->resSam) = Initialize(sam_encoder, sam_decoder, backend);

    is_initialized_ = true;
}

SegmentationResult SamSegPipeline::process(const cv::Mat& img, const cv::Mat& depth_image, const std::string& ignore_label, bool verbose)
{
    SegmentationResult seg_result;

    if (!is_initialized_) {
        ROS_WARN_THROTTLE(1.0, "SamSegPipeline is currently initializing in the background. Skipping frame...");
        return seg_result;
    }

    // Total pipeline timing
    auto pipeline_start = std::chrono::high_resolution_clock::now();

    ////////////////////////// YOLO INFERENCE //////////////////////////////////////
    auto yolo_infer_start = std::chrono::high_resolution_clock::now();

    std::vector<DL_RESULT> resYolo;
    resYolo = Detector(pimpl_->yoloDetector, img);

    auto yolo_infer_end = std::chrono::high_resolution_clock::now();

    ////////////////////////// SAM INFERENCE //////////////////////////////////////
    pimpl_->sam_res.boxes.clear();
    pimpl_->resSam.clear();

    for (const auto& result : resYolo)
    {
        std::string yolo_class = pimpl_->yoloDetector->classes[result.classId];

        // Filter by ignore_label (typically the supporting surface itself)
        if (!ignore_label.empty() && yolo_class == ignore_label)
        {
            ROS_DEBUG("Pre-filtering box for '%s' (matches ignore_label '%s')", yolo_class.c_str(), ignore_label.c_str());
            continue;
        }

        // Filter out boxes containing 0 valid points within the target volume frustum
        if (!depth_image.empty())
        {
            int x = std::max(0, result.box.x);
            int y = std::max(0, result.box.y);
            int w = std::min(depth_image.cols - x, result.box.width);
            int h = std::min(depth_image.rows - y, result.box.height);

            if (w > 0 && h > 0)
            {
                cv::Mat box_depth = depth_image(cv::Rect(x, y, w, h));
                // filter it out is if it contains less than 30% of valid depth points (non-zero, finite) — this is a heuristic to skip boxes that are mostly outside the sensor frustum - if its too aggressive we can always lower the threshold
                if (cv::countNonZero(box_depth > 0.0f) < 0.3 * w * h)
                {
                    ROS_DEBUG("Pre-filtering box for '%s' (less than 30%% depth points within frustum volume)", yolo_class.c_str());
                    continue;
                }
            }
        }

        pimpl_->sam_res.boxes.push_back(result.box);
        seg_result.labels.push_back(yolo_class);
        seg_result.confidences.push_back(result.confidence);
        ROS_DEBUG("Confidence: %f", result.confidence);
        ROS_DEBUG("Class: %s", yolo_class.c_str());
        ROS_DEBUG("Class ID: %d", result.classId);
    }

    auto sam_infer_start = std::chrono::high_resolution_clock::now();

    SegmentAnything(pimpl_->samWrapper, pimpl_->sam_encoder_params, pimpl_->sam_decoder_params, img, pimpl_->resSam, pimpl_->sam_res);

    auto sam_infer_end = std::chrono::high_resolution_clock::now();

    auto pipeline_end = std::chrono::high_resolution_clock::now();

    // Compute latency measurements if requested
    if (verbose)
    {
        seg_result.latency.yolo_init_ms = 0.0;
        seg_result.latency.yolo_infer_ms = std::chrono::duration<double, std::milli>(yolo_infer_end - yolo_infer_start).count();
        seg_result.latency.sam_init_ms = 0.0;
        seg_result.latency.sam_infer_ms = std::chrono::duration<double, std::milli>(sam_infer_end - sam_infer_start).count();
        seg_result.latency.total_pipeline_ms = std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();

        ROS_WARN("\n=== Segmentation Pipeline Latency ===\n");
        ROS_WARN("  YOLO infer:     %7.2f ms (%zu bboxs)\n", seg_result.latency.yolo_infer_ms, resYolo.size());
        ROS_WARN("  SAM infer:      %7.2f ms (%zu masks)\n", seg_result.latency.sam_infer_ms, pimpl_->resSam.empty() ? 0 : pimpl_->resSam.front().masks.size());
        ROS_WARN("  Total pipeline: %7.2f ms\n", seg_result.latency.total_pipeline_ms);
    }

    if (!pimpl_->resSam.empty())
    {
        seg_result.masks = std::move(pimpl_->resSam.front().masks);
    }
    seg_result.boxes = std::move(pimpl_->sam_res.boxes);


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

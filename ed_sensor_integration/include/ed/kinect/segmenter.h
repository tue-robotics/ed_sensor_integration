#ifndef ED_SENSOR_INTEGRATION_SEGMENTER_H_
#define ED_SENSOR_INTEGRATION_SEGMENTER_H_

#include "ed/kinect/entity_update.h"
#include <ed_sensor_integration/kinect/segmodules/sam_seg_module.h>

#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>
#include <tue/config/configuration.h>

#include <ed/convex_hull.h>
#include <ed/types.h>

#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

namespace cv
{
    class Mat;
}

namespace geo
{
    class DepthCamera;
}

// ----------------------------------------------------------------------------------------------------

class Segmenter
{

public:

    Segmenter(tue::Configuration config);

    ~Segmenter();

    void removeBackground(cv::Mat& depth_image, const ed::WorldModel& world, const geo::DepthCamera& cam,
                          const geo::Pose3D& sensor_pose, double background_padding);

    void calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                               const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image) const;
    /**
     * @brief Preprocess RGB image for segmentation. This function masks the RGB image with the filtered depth image (non-zero depth values are kept)
     * It returns a masked RGB image with values only where depth is non-zero
     *
     * @param rgb_image
     * @param filtered_depth_image
     * @return cv::Mat filtered_depth_image
     */
    cv::Mat preprocessRGBForSegmentation(const cv::Mat& rgb_image, const cv::Mat& filtered_depth_image) const;

    /**
     * @brief Cluster the depth image into segments. Applies new algorithm which is the YOLO - SAM - BMM depth segmentation pipeline.
     *
     * @param depth_image
     * @param cam_model
     * @param sensor_pose
     * @param clusters
     * @param rgb_image
     * @param verbose
     * @param area_description  Area description from the ROS service request (e.g. "on_top_of dinner_table").
     *                          When the area name is "on_top_of", YOLO-detected supporting surfaces are
     *                          skipped before any point extraction or BMM.  The mapping from the ED entity
     *                          name (e.g. "dinner_table") to the YOLO class label (e.g. "dining table") is
     *                          read from the "surface_label_map" array in the segmenter config block.
     * @return SegmentationResult containing masks, bounding boxes, labels, and confidences
     */
    SegmentationResult cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                 const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters, const cv::Mat& rgb_image,
                const std::string& area_description = "", bool verbose=false);

private:
    tue::Configuration config_;
    /// Maps ED entity names to their Neural Network Classifier (YOLO) class label (acts as a lookup table).
    /// Populated from the "surface_label_map" array in world_model_plugin_rgbd.yaml (entity: key + yolo_label: value).
    std::unordered_map<std::string, std::string> surface_label_map_;
};

#endif

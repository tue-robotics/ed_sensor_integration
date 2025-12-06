#ifndef ED_SENSOR_INTEGRATION_SEGMENTER_H_
#define ED_SENSOR_INTEGRATION_SEGMENTER_H_

#include "ed/kinect/entity_update.h"

#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>
#include <tue/config/configuration.h>

#include <ed/convex_hull.h>
#include <ed/types.h>

#include <vector>

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
     * @return std::vector<cv::Mat> masks // 3D pointcloud masks of all the segmented objects
     */
    std::vector<cv::Mat> cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                 const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters, const cv::Mat& rgb_image, bool logging);

private:
    tue::Configuration config_;
};

#endif

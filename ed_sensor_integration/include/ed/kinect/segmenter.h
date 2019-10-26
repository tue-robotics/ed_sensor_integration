#ifndef ED_SENSOR_INTEGRATION_SEGMENTER_H_
#define ED_SENSOR_INTEGRATION_SEGMENTER_H_

#include "ed/kinect/entity_update.h"

#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

#include <ed/convex_hull.h>
#include <ed/types.h>

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

    Segmenter();

    ~Segmenter();

    void removeBackground(cv::Mat& depth_image, const ed::WorldModel& world, const geo::DepthCamera& cam,
                          const geo::Pose3D& sensor_pose, double background_padding);

    void calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                               const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image) const;

    void cluster(const cv::Mat& depth_image, const geo::DepthCamera& cam_model,
                 const geo::Pose3D& sensor_pose, std::vector<EntityUpdate>& clusters) const;

private:

};

#endif

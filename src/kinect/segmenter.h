#ifndef ED_SENSOR_INTEGRATION_SEGMENTER_H_
#define ED_SENSOR_INTEGRATION_SEGMENTER_H_

#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

namespace cv
{
    class Mat;
};

class Segmenter
{

public:

    Segmenter();

    ~Segmenter();

    void calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                               const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image);

private:

};

#endif

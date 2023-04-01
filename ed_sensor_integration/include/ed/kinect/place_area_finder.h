#ifndef ED_KINECT_PLACE_AREA_FINDER_H_
#define ED_KINECT_PLACE_AREA_FINDER_H_


#include <rgbd/image.h>
#include <geolib/datatypes.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

class PlaceAreaFinder
{

public:

    PlaceAreaFinder();

    ~PlaceAreaFinder();

    void findArea(const rgbd::ImageConstPtr& image, geo::Pose3D sensor_pose);

    void getCanvas(cv::Mat& image){ canvas.copyTo(image); }

private:
    cv::Mat canvas;
};

#endif

#ifndef ED_KINECT_PLACE_AREA_FINDER_H_
#define ED_KINECT_PLACE_AREA_FINDER_H_


#include <rgbd/image.h>
#include <geolib/datatypes.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PlaceAreaFinder
{

public:

    PlaceAreaFinder();

    ~PlaceAreaFinder();

    bool findArea(const rgbd::ImageConstPtr& image, geo::Pose3D sensor_pose, geo::Pose3D& place_pose);

    void getCanvas(cv::Mat& image){ canvas.copyTo(image); }

private:
    // internal occupancy representation
    double resolution = 0.005;
    cv::Point2d canvas_center;
    cv::Mat canvas;

    cv::Point2d worldToCanvas(double x, double y);
    geo::Vec2d canvasToWorld(cv::Point2d point);
    void createCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Scalar color);
    void closeCanvas(cv::Mat& canvas, cv::Mat& closed_canvas, float placement_margin);
    void alterPlane(cv::Mat& closed_canvas, cv::Mat& smallplane_canvas, float placement_margin);
    void createRadiusCostmap(cv::Mat& canvas, cv::Scalar color, float placement_margin);
    void dilateCostmap(cv::Mat& canvas, cv::Mat& dilated_canvas, float placement_margin);
};

#endif

#ifndef ED_KINECT_PLACE_AREA_FINDER_H_
#define ED_KINECT_PLACE_AREA_FINDER_H_


#include <rgbd/image.h>
#include <geolib/datatypes.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Tool for finding placement options using rgbd sensor information.
 *
 */
class PlaceAreaFinder
{

public:

    PlaceAreaFinder();

    ~PlaceAreaFinder();


    /**
     * @brief find a horizontal surface suitable to place an object
     *
     * @param image image in which to find a place position
     * @param sensor_pose pose of the sensor with respect to a horizontal plane 'base_link' recommended
     * @param[out] place_pose one of the possible poses where an object may be placed
     * @return whether or not a suitable place was found
     */
    bool findArea(const rgbd::ImageConstPtr& image, geo::Pose3D sensor_pose, geo::Pose3D& place_pose);

    /**
     * @brief Get an image of the analysed space, used for introspection
     *
     * @param[out] image canvas to write the image to.
     */
    void getCanvas(cv::Mat& image){ canvas.copyTo(image); }

private:
    // internal occupancy representation
    double resolution = 0.005;
    cv::Point2d canvas_center;
    cv::Mat canvas;

    /**
     * @brief transform a point in meters to a pixel on the canvas
     *
     * @param x in meters
     * @param y in meters
     * @return cv::Point2d
     */
    cv::Point2d worldToCanvas(double x, double y);

    /**
     * @brief inverse of WorldToCanvas
     *
     * @param point point on the canvas
     * @return geo::Vec2d matching point in 2D space
     */
    geo::Vec2d canvasToWorld(cv::Point2d point);

    /**
     * @brief Draw the points in a pointcloud on the canvas. will ignore the z direction of the points
     *
     * @param cloud pointcloud to be rendered
     * @param color color to render the points
     */
    void createCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Scalar color);

    /**
     * @brief morphological closing operation
     *
     * @param canvas image to close
     * @param closed_canvas image after closing
     * @param placement_margin radius for closing in meters! will be converted internally. #TODO remove
     */
    void closeCanvas(cv::Mat& canvas, cv::Mat& closed_canvas, float placement_margin);

    /**
     * @brief morphological dilation
     *
     * @param canvas image to dilate
     * @param dilated_canvas image after dilation
     * @param placement_margin radius for dilation in meters! will be converted internally. #TODO remove
     */
    void dilateCostmap(cv::Mat& canvas, cv::Mat& dilated_canvas, float placement_margin);

    /**
     * @brief not a clue, appears to be an erosion operation
     *
     * @param closed_canvas
     * @param smallplane_canvas
     * @param placement_margin
     */
    void alterPlane(cv::Mat& closed_canvas, cv::Mat& smallplane_canvas, float placement_margin);

    /**
     * @brief Draw a Circle on the canvas indicating the prefered distance to the robot. This distance is currently hardcoded.
     *
     * @param canvas canvas to draw on
     * @param color color to draw with
     * @param placement_margin width of the semi-circle.
     */
    void createRadiusCostmap(cv::Mat& canvas, cv::Scalar color, float placement_margin);

};

#endif

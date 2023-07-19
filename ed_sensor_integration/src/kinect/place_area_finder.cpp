#include "ed/kinect/place_area_finder.h"

#include <iostream>
#include <string>

#include <vector>

#include <ros/ros.h>

//pcl library # TODO remove the unused ones #TODO find out which ones are unused
#include <pcl/point_representation.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/surface/convex_hull.h>

#include "ed_sensor_integration/sac_model_horizontal_plane.h"


/**
 * @brief transform an rgbd image to a pointcloud
 * 
 * @param image rgbd image
 * @param[out] cloud pcl pointcloud, points are expressed in the frame of the camera according to pcl conventions
 * @return double to be removed
 */
double imageToCloud(const rgbd::Image& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Fill in the cloud data
    cloud->width = image.getDepthImage().cols;
    cloud->height = image.getDepthImage().rows;
    cloud->is_dense = false;
    cloud->resize (cloud->width * cloud->height);

    double fx = image.getCameraModel().fx();
    double fy = image.getCameraModel().fy();

    double half_height = 0.5 * cloud->height;
    double half_width = 0.5 * cloud->width;

    for (uint i=0; i < cloud->height; ++i)
    {
        for (uint j=0; j < cloud->width; ++j)
        {

            cv::Vec3b bgr = image.getRGBImage().at<cv::Vec3b>(i,j);
            double d = image.getDepthImage().at<float>(i,j);

            cloud->at(j,i).x = (-half_width+j) * d / fx;
            cloud->at(j,i).y = (-half_height+i) * d / fy;
            cloud->at(j,i).z = d;
            cloud->at(j,i).r = bgr[2];
            cloud->at(j,i).g = bgr[1];
            cloud->at(j,i).b = bgr[0];

        }
    }

return image.getCameraModel().fx();
}

/**
 * @brief transform a geolib pose into a transformation matrix from the Eigen library
 * 
 * @param pose geolib pose
 * @return Eigen::Matrix4f 
 */
Eigen::Matrix4f geolibToEigen(geo::Pose3D pose)
{
    // convert from geolib coordinates to ros coordinates #TODO remove geolib coordinates for camera pose
    pose.R = pose.R * geo::Mat3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    float x = pose.t.x;
    float y = pose.t.y;
    float z = pose.t.z;
    float xx = pose.R.xx;
    float xy = pose.R.xy;
    float xz = pose.R.xz;
    float yx = pose.R.yx;
    float yy = pose.R.yy;
    float yz = pose.R.yz;
    float zx = pose.R.zx;
    float zy = pose.R.zy;
    float zz = pose.R.zz;

    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();

    Transform(0,0) = xx;
    Transform(0,1) = xy;
    Transform(0,2) = xz;
    Transform(0,3) = x;
    Transform(1,0) = yx;
    Transform(1,1) = yy;
    Transform(1,2) = yz;
    Transform(1,3) = y;
    Transform(2,0) = zx;
    Transform(2,1) = zy;
    Transform(2,2) = zz;
    Transform(2,3) = z;

    // std::cout << Transform << std::endl;
    return Transform;
}

/**
 * @brief segment the larges horizontal plane in the pointcloud. Horizontal is assumed to be normal to the z-axis. The plane is also assumed to be above z=0.0
 * @param cloud_in pointcloud to be segmented
 * @param plane_coefficients the 4 coefficients that define a parallel plane
 * @param cloud_out A pointcloud containing all points within the found plane
 * @param cloud_no_plane A pointcloud containing all points not within the found plane
 * @return height of the segmented plane OR -1.0 if no plane could be found
 */
bool SegmentPlane (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::ModelCoefficients::Ptr plane_coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_plane)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setAxis (Eigen::Vector3f(1,0,0));
    seg.setEpsAngle(5*0.0174532925); //*0.0174532925 to radians

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract2;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *plane_coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return false;
    }

    // Extract the inliers to a cloud with just the plane
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);

    // std::cout << "PointCloud representing the planar component: " << inliers->indices.size() << " data points."
    //   << "Plane with coefficients: " << *coefficients << std::endl;

    // Extract outliers to the main cloud without the table plane
    extract2.setInputCloud(cloud_in);
    extract2.setIndices(inliers);
    extract2.setNegative(true);
    extract2.filter(*cloud_no_plane);

    return true;
}


/**
 * @brief extract all pixels of a certain color from a canvas
 * 
 * @param canvas original image
 * @param[out] placement_canvas image to draw the extracted pixels on must be the same size as canvas 
 * @param targetColor color to be extracted
 */
void ExtractPlacementOptions(cv::Mat& canvas, cv::Mat& placement_canvas, cv::Scalar targetColor)
{
    cv::Point2d canvas_center(canvas.rows / 2, canvas.cols);

    std::vector<cv::Point> identicalPoints;

    for(int row = canvas.rows; row > 0; --row)
    {
        for(int col = canvas.cols; col > 0; --col)
        {
            cv::Vec3b currPixel = canvas.at<cv::Vec3b>(row, col);
            if(currPixel.val[0] == targetColor.val[0] &&
               currPixel.val[1] == targetColor.val[1] &&
               currPixel.val[2] == targetColor.val[2])
            {
                cv::Point2d p = cv::Point(col, row);
                if (p.x >= 0 && p.y >= 0 && p.x < placement_canvas.cols && p.y < placement_canvas.rows)
                    placement_canvas.at<cv::Vec3b>(p) = cv::Vec3b(targetColor[0], targetColor[1], targetColor[2]);
            }
        }
    }
}

/**
 * @brief Get the coordinates of a point in an image which matches the target color
 * 
 * @param canvas image to check
 * @param targetColor color to be found
 * @param point coordinates of one point which has the targetColor
 * @return whether or not a pixel was found
 */
bool GetPlacementOption(cv::Mat& canvas, cv::Scalar targetColor, cv::Point2d& point)
{
    for(int row = 0; row <canvas.rows; ++row)
    {
        for(int col = canvas.cols; col > 0; --col)
        {
            cv::Vec3b currPixel = canvas.at<cv::Vec3b>(row, col);
            if(currPixel.val[0] == targetColor.val[0] &&
               currPixel.val[1] == targetColor.val[1] &&
               currPixel.val[2] == targetColor.val[2])
            {
                point = cv::Point(col, row);
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief remove the floor from a pointcloud cloud_in and return the cloud_out with index. It is assumed the floor is aligned with the plane z=0
 * 
 * @param cloud_in 
 * @param cloud_out 
 * @param index 
 */
void removeFloor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::Indices &index)
{
    float buffer = 0.1;
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, buffer)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_in);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter(*cloud_out);
    (*cloud_out).is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, index);
}

void filterHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float height_min, float height_max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::Indices &index)
{
    // Filter out objects above the table plane and put them in seperate cloud
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, height_min)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, height_max)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem2;
    condrem2.setCondition(range_cond);
    condrem2.setInputCloud(cloud_in);
    condrem2.setKeepOrganized(true);
    // apply filter
    condrem2.filter(*cloud_out);
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, index);
}

/**
 * @brief Project a plane onto a a plane along the beams of the camera
 * 
 * @param cloud_in cloud with points to be projected
 * @param transform transform between the camera and the origin of the cloud in
 * @param plane_height height of the plane to be projected onto.
 * @param cloud_out cloud containing points with z=height
 */
void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Eigen::Matrix4f transform, float plane_height, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
    cloud_out->width = cloud_in->width;
    cloud_out->height = cloud_in->height;
    cloud_out->is_dense = cloud_in->is_dense;
    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    float x = transform(0, 3);
    float y = transform(1, 3);
    float z = transform(2, 3);
    for (uint nIndex = 0; nIndex < cloud_in->points.size(); nIndex++)
    {
        float lower = cloud_in->points[nIndex].z - z; // height between camera and point. 
        float upper = plane_height - cloud_in->points[nIndex].z; // height between the point and the table
        float lambda = upper / lower; // ratio between camera-point distance and point-projection distance
        float dx = cloud_in->points[nIndex].x - x; // difference between point x and camera x
        float dy = cloud_in->points[nIndex].y - y; // difference between point y and camera y

        cloud_out->points[nIndex].z = plane_height;
        cloud_out->points[nIndex].x = cloud_in->points[nIndex].x + lambda * dx;
        cloud_out->points[nIndex].y = cloud_in->points[nIndex].y + lambda * dy;
    }
}

PlaceAreaFinder::PlaceAreaFinder()
{
}

PlaceAreaFinder::~PlaceAreaFinder()
{
}

bool PlaceAreaFinder::findArea(const rgbd::ImageConstPtr& image, geo::Pose3D sensor_pose, geo::Pose3D& place_pose)
{
    // std::cout << "converting image to cloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    imageToCloud(*image, cloud);

    // transform to base link frame
    Eigen::Matrix4f transform = geolibToEigen(sensor_pose);
    pcl::transformPointCloud(*cloud, *cloud, transform);

    // keep track of the indices in the original image
    std::vector<int> indices;
    pcl::Indices floorless_index;

    // Filter out floor
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr floorless_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    removeFloor(cloud, floorless_cloud, floorless_index);

    // Segment the table plane and return a cloud with the plane and a cloud where the plane is removed
    std::cout << "SegmentPlane" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeless_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());
    if (!SegmentPlane(floorless_cloud, plane_coefficients, plane_cloud, planeless_cloud))
    {
        std::cout << "Could not find plane" << std::endl;
        return false;
    }
    float height = abs(plane_coefficients->values[3]); // take the absolute value in case the plane is fitted upside down
    std::cout << "Found plane height " << height << std::endl;

    // Filter out objects above the table plane and put them in seperate cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::Indices object_index;
    filterHeight(planeless_cloud, height, height+0.30, object_cloud, object_index);

    // Create pointcloud with occluded space based on the object cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occluded_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectToPlane(object_cloud, transform, height, occluded_cloud);

    // Filter out objects below the table plane and put them in seperate cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr below_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::Indices below_index;
    filterHeight(planeless_cloud, 0.0, height-0.02, below_table_cloud, below_index);

    // Create pointcloud with unoccluded space based on the bewow table cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectToPlane(below_table_cloud, transform, height, not_table_cloud);    

    // std::cout << "creating costmap" << std::endl;
    canvas = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    dilated_canvas = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    placement_canvas = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar table_color(0, 255, 0);
    cv::Scalar occupied_color(0, 0, 255);
    cv::Scalar occluded_color(255, 0, 0);
    cv::Scalar radius_color(100, 0, 100);
    cv::Scalar placement_color(100, 255, 100);
    cv::Scalar point_color(255, 0, 0);

    // Object placement margins #TODO hardcoded parameters
    float object_diameter = 0.10;
    float error_margin = 0.02;
    float length = object_diameter + error_margin;
    float placement_margin = 2 * 0.02 + length;

    std::cout << "annotating image" << std::endl;
    annotated_image = image->getRGBImage().clone();
    annotateImage(*image, floorless_index, table_color);

    std::cout << "creating costmap" << std::endl;

    // draw clouds to costmap
    createCostmap(plane_cloud, table_color);
    createCostmap(object_cloud, occupied_color);
    createCostmap(occluded_cloud, occluded_color);
    createCostmap(not_table_cloud, occupied_color);

    // HERO preferred radius
    createRadiusCostmap(canvas, radius_color, placement_margin);

    // Dilate the costmap and create a new canvas
    dilateCostmap(canvas, dilated_canvas, placement_margin);

    ExtractPlacementOptions(dilated_canvas, placement_canvas, table_color);

    // Extract the placement options and choose a placement solution
    cv::Point2d place_point_canvas;
    if (!GetPlacementOption(dilated_canvas, table_color, place_point_canvas))
    {
        return false;
    }

    geo::Vec2d place_point;
    place_point = canvasToWorld(place_point_canvas);

    // fill result
    place_pose = geo::Pose3D(place_point.x, place_point.y, height+0.02);
    return true;
}

cv::Point2d PlaceAreaFinder::worldToCanvas(double x, double y)
{
    return cv::Point2d(-y / resolution, -x / resolution) + canvas_center;
}

geo::Vec2d PlaceAreaFinder::canvasToWorld(cv::Point2d point)
{
    double y = (point.x-canvas_center.x)*-resolution;
    double x = (point.y-canvas_center.y)*-resolution;
    return geo::Vec2d(x, y);
}

void PlaceAreaFinder::createCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Scalar color)

{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (uint nIndex = 0; nIndex < cloud->points.size (); nIndex++)
    {
        double x = cloud->points[nIndex].x;
        double y = cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }    
}

void PlaceAreaFinder::createRadiusCostmap(cv::Mat& canvas, cv::Scalar color, float placement_margin)
{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
    float upper_radius = 0.75 + placement_margin/2;
    float lower_radius = 0.60 - placement_margin/2;
        for (float phi = 0; phi < 360; phi++)
        {
            for (int i = 0; i < 100; i++)
            {
            double y = upper_radius * sin(phi/(180/M_PI)) + 0.03 * i * sin(phi/(180/M_PI));
            double x = upper_radius * cos(phi/(180/M_PI)) + 0.03 * i * cos(phi/(180/M_PI));

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
            }

            for (int i = 0; i < 100; i++)
            {
            double y = lower_radius * sin(phi/(180/M_PI)) - lower_radius/100 * i * sin(phi/(180/M_PI));
            double x = lower_radius * cos(phi/(180/M_PI)) - lower_radius/100 * i * cos(phi/(180/M_PI));

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
}

void PlaceAreaFinder::dilateCostmap(cv::Mat& canvas, cv::Mat& dilated_canvas, float placement_margin)
{
    float Pixelsize = placement_margin / resolution;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::dilate(canvas, dilated_canvas, element );
}

void PlaceAreaFinder::annotateImage(const rgbd::Image& image, const pcl::Indices index, cv::Scalar color)
{
    int width = image.getDepthImage().cols;
    
    for (uint i = 0; i < index.size(); i++)
    {
        int col = index[i] % width;
        int row = index[i] / width;

        cv::Point2d p = cv::Point2d(col, row);
        if (p.x >= 0 && p.y >= 0 && p.x < annotated_image.cols && p.y < annotated_image.rows)
            annotated_image.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }    
}

#include <iostream>
#include <string>

#include <rgbd/image_buffer/image_buffer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

//
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//
#include <ros/ros.h>
#include <rgbd/image.h>

//pcl library # TODO remove the unused ones #TODO find out which ones are unused
#include <pcl/point_cloud.h>
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

#include <ed/kinect/place_area_finder.h>


double resolution = 0.005;
cv::Point2d canvas_center;
cv::Mat mask; // global variable for a\passing the image mask
std::mutex mutex;

geo::Vector3 simpleRayTrace(geo::Vector3 origin, geo::Vector3 direction)
{
    if (direction.z > 0)
    {
        std::cout << "Raytrace went terribly wrong" << std::endl;
    }
    double ratio = -origin.z / direction.z;

    return geo::Vector3(origin.x + ratio * direction.x, origin.y + ratio * direction.y, 0);
}

void drawFoVMacro(geo::Vector3 direction, cv::Mat& canvas, geo::Pose3D sensor_pose)
{
    // convert vectors to world frame
    geo::Vector3 direction_world = sensor_pose.R * direction;

    if (direction_world.z > 0.0)
    {
        std::cout << "above plane" << std::endl;
        return;
    }

    // project vectors on place
    geo::Vector3 projected_point = simpleRayTrace(sensor_pose.t, direction_world);

    // draw projected points
    cv::Point2d p1_canvas = cv::Point2d(-projected_point.y / resolution, -projected_point.x / resolution) + canvas_center;
    cv::Scalar fovcolor(0, 255, 255); // Red
    cv::circle(canvas, p1_canvas, 5, fovcolor, -1);

    std::cout << "direction in camera frame: " << direction << std::endl;
    std::cout << "direction in world frame: " << direction_world << std::endl;
    std::cout << "projection in world frame: " << projected_point << std::endl;
    std::cout << "cvpoint in canvas frame: x: " << p1_canvas.x << ", y: " << p1_canvas.y << std::endl;
}
/**
 * @brief Draw the fiel of view of the camera on a plane.
 * 
 * @param canvas image representing a plane
 * @param sensor_pose pose of the camera with respect to the plane. It is assumed that the origin corresponds to the canvas_center point on the plane.
 * @param caminfo Used to get focal lengths of the camera
 */
void drawFieldOfView(cv::Mat& canvas, geo::Pose3D sensor_pose, const rgbd::ImageConstPtr& caminfo)
{
    // Get camera info
    int width = caminfo->getDepthImage().cols;
    int height = caminfo->getDepthImage().rows;
    double half_height = 0.5*height;
    double half_width = 0.5*width;

    double fx = caminfo->getCameraModel().fx();
    double fy = caminfo->getCameraModel().fy();

    // determine vectors pointing to corners of FoV
    geo::Vector3 c0(0, 0, -1.0); // upper left of image
    geo::Vector3 c1(half_width / fx, half_height / fy, -1.0); // upper left of image
    geo::Vector3 c2(-half_width / fx, half_height / fy, -1.0); // upper right of image
    geo::Vector3 c3(-half_width / fx, -half_height / fy, -1.0); // lower right of image
    geo::Vector3 c4(half_width / fx, -half_height / fy, -1.0); // lower left of image

    // draw
    std::cout << "center" << std::endl;
    drawFoVMacro(c0, canvas, sensor_pose);
    std::cout << "upper left" << std::endl;
    drawFoVMacro(c1, canvas, sensor_pose);
    std::cout << "upper right" << std::endl;
    drawFoVMacro(c2, canvas, sensor_pose);
    std::cout << "lower right" << std::endl;
    drawFoVMacro(c3, canvas, sensor_pose);
    std::cout << "lower left" << std::endl;
    drawFoVMacro(c4, canvas, sensor_pose);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "Mask received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  try
  {
    if(mutex.try_lock())
    {
        mask = cv_bridge::toCvCopy(msg, "8UC3")->image;
        std::cout << "Mask received" << std::endl;
        mutex.unlock();
    }
    else
    {
        std::cout << "Mutex not locked!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
    //cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @brief usage, print how the executable should be used and explain the input
 */
void usage()
{
    std::cout << "Usage: ed_empty_spot_designator RGBD_TOPIC BOOL" << std::endl
              << "RGBD_TOPIC topic on which the rgbd image is published, example /hero/head_rgbd_sensor/rgbd" << std::endl
              << "Set second argument to donal or max to run that version of the code" << std::endl;
}

void drawMaskContour(cv::Mat& image, const cv::Mat& mask)
{
    // Convert the mask to grayscale if it's not already
    cv::Mat grayMask;
    if (mask.channels() > 1)
    {
        cv::cvtColor(mask, grayMask, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayMask = mask.clone(); // If it's already single-channel, create a copy
    }

    // Find contours in the binary mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(grayMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw the contours on the image in white
    cv::Scalar contourColor(255, 255, 255); // White color
    cv::drawContours(image, contours, -1, contourColor, 2); // -1 means draw all contours, 2 is the thickness
}

rgbd::ImageConstPtr createModifiedImage(const rgbd::ImageConstPtr& originalImagePtr, const cv::Mat& newRGBValues) {
    rgbd::Image modifiedImage = originalImagePtr->clone();
    modifiedImage.setRGBImage(newRGBValues);
    rgbd::ImageConstPtr modifiedImagePtr = std::make_shared<const rgbd::Image>(modifiedImage);

    return modifiedImagePtr;
}

/**
 * @brief main executable to visualise the empty spot finder of live images.
 * @param argc:
 * @return
 */
int main (int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 0;
    }

    ros::init(argc, argv, "empty_spot_visualizer");

    std::string topic = argv[1];
    std::cout << "Using topic: " << topic << std::endl;

    std::string arg = argv[2];
    bool donal = (arg == "donal");
    if (donal){
        std::cout <<"Running Donal's version!"<<std::endl;
    }
    else{
        std::cout<<"Running Max's version!"<<std::endl;
    }

    rgbd::ImageBuffer image_buffer;
    image_buffer.initialize(topic, "base_link");

    pcl::PCDWriter writer;

    PlaceAreaFinder place_area_finder;
    //
    std::string mask_topic = "/hero/segmented_image";
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(mask_topic, 1, imageCallback);

    //
    while(ros::ok())
    {
        rgbd::ImageConstPtr image;
        geo::Pose3D sensor_pose;
        geo::Pose3D place_pose;


        if (!image_buffer.waitForRecentImage(image, sensor_pose, 2.0))
        {
            std::cerr << "No image received, will try again." << std::endl;
            continue;
        }

        std::cout << "Trying to replace RGB with mask" << std::endl;
        if(!mask.empty() && donal)
        {
            if(mutex.try_lock()){
                rgbd::ImageConstPtr new_image_ptr = createModifiedImage(image, mask.clone());
                cv::Mat rgbcanvas = new_image_ptr->getRGBImage();
                // cv::imshow("RGB remapped to mask", rgbcanvas);
                std::cout << "Replaced RGB with mask!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

                if (!place_area_finder.findArea(new_image_ptr, sensor_pose, place_pose,mask,donal))
                {
                    std::cout << "no place area found" << std::endl;
                }
                mutex.unlock();
            }
            
        }
        else{
            if (!place_area_finder.findArea(image, sensor_pose, place_pose,mask,donal))
                {
                    std::cout << "no place area found" << std::endl;
                }
        }
            
        
        std::cout << place_pose << std::endl;

        cv::Mat canvas;
        cv::Mat dilated_canvas;
        cv::Mat placement_canvas;
        place_area_finder.getCanvas(canvas);
        // place_area_finder.getDilatedCanvas(dilated_canvas);
        // place_area_finder.getPlacementCanvas(placement_canvas);
        /*
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        geo::Pose3D sensor_pose_canvas = sensor_pose;
        sensor_pose_canvas.t.z = sensor_pose.t.z - place_pose.t.z;
        drawFieldOfView(canvas, sensor_pose_canvas, image);
        */
        // Show the different canvasses
        
        // std::cout << "showing costmap" << std::endl;
        if(!canvas.empty()){
        cv::imshow("Costmap topview", canvas);
        }
                
        // std::cout << "showing dilated costmap" << std::endl;
        if(!dilated_canvas.empty()){
        cv::imshow("Dilated costmap topview", dilated_canvas);
        }
        // std::cout << "showing placement costmap" << std::endl;
        if(!placement_canvas.empty()){
        cv::imshow("Placement options costmap topview", placement_canvas);
        }
        // if (mutex.try_lock()){

        //     if (!mask.empty()) {
        //     cv::imshow("Mask", mask);
        //     }
        //     mutex.unlock();
        // }
        if (!mask.empty())
        {
            cv::Mat annotated_image;
            cv::Mat annotated_image_with_mask;
            place_area_finder.getAnnotatedImage(annotated_image);
            annotated_image_with_mask = annotated_image.clone();
            drawMaskContour(annotated_image_with_mask, mask);
            cv::imshow("Annotated_image", annotated_image_with_mask);
        }
        // Show RGB snapshot
        cv::Mat rgbcanvas = image->getRGBImage();
        cv::imshow("Original RGB", rgbcanvas);
        drawMaskContour(rgbcanvas, mask);
        cv::imshow("Original RGB with Table Mask", rgbcanvas);

        char key = cv::waitKey(30);

        if (key == 'q')
        {
            break;
        }
        ros::spinOnce();
    }
    cv::destroyAllWindows();
    return 0;
}

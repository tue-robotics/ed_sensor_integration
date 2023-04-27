#include <iostream>
#include <string>

#include <rgbd/image_buffer/image_buffer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

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
<<<<<<< HEAD
        std::cout << "above plane" << std::endl;
=======
        double x = cloud->points[nIndex].x;
        double y = cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }

    
}

void createObjectCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud, cv::Mat& canvas, cv::Scalar color)

{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (int nIndex = 0; nIndex < object_cloud->points.size (); nIndex++)
    {
        double x = object_cloud->points[nIndex].x;
        double y = object_cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }

}

void createOccludedCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr occluded_cloud, cv::Mat& canvas, cv::Scalar color)

{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (int nIndex = 0; nIndex < occluded_cloud->points.size (); nIndex++)
    {
        double x = occluded_cloud->points[nIndex].x;
        double y = occluded_cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }   
}

void createNotTableCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr notTable_cloud, cv::Mat& canvas, cv::Scalar color)

{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (int nIndex = 0; nIndex < notTable_cloud->points.size (); nIndex++)
    {
        double x = notTable_cloud->points[nIndex].x;
        double y = notTable_cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }
}

void createFOVLCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y, double fx)

{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        for (int nIndex = 0; nIndex < 3000 ; nIndex++)
        {
        float initial_x = x;
        float initial_y = y;
        double y = initial_y + (0.001)*nIndex;
        double x = initial_x + 0.001*nIndex*tan(60/(180/M_PI));


        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
        }
}

void createFOVRCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y)

{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        for (int nIndex = 0; nIndex < 3000 ; nIndex++)
        {
        float initial_x = x;
        float initial_y = y;
        double y = initial_y - 0.001*nIndex;
        double x = initial_x + 0.001*nIndex*tan(60/(180/M_PI));

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
        }
}

void createFOVHCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y, float z, float height)
{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        for (int i = 0; i < 100; i++)
        {
            for (int nIndex = 0; nIndex < 4000; nIndex++)
            {
            float initial_x = x;
            float initial_y = y;
            double y = initial_y + -2+ 0.001*nIndex;
            double x = (initial_x + (z-height)*tan(67.0/(180/M_PI))); // 67.5 deg

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
                canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
}

void CloseCanvas(cv::Mat& canvas, cv::Mat& closed_canvas, float placement_margin)
{
    float Pixelsize = 5*(placement_margin / resolution);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::dilate(canvas, closed_canvas, element );
    cv::erode(closed_canvas, closed_canvas, element);
}


void AlterPlane(cv::Mat& closed_canvas, cv::Mat& smallplane_canvas, float placement_margin)
{
    float Pixelsize = 2*(placement_margin / resolution);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::erode(closed_canvas, smallplane_canvas, element);
}




void createRadiusCostmap(cv::Mat& canvas, cv::Scalar color, float placement_margin)
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

void dilateCostmap(cv::Mat& canvas, float placement_margin)
{
    float Pixelsize = placement_margin / resolution;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::Scalar lower_red = cv::Scalar(0,0,100);
    cv::Scalar upper_red = cv::Scalar(0,0,255);
    cv::Mat red_mask;
    cv::inRange(canvas, lower_red, upper_red, red_mask);
    cv::Mat dilated_canvas;
    cv::dilate(canvas, dilated_canvas, element, cv::Point(-1,-1),1,cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
    dilated_canvas.setTo(cv::Scalar(0,0,255),red_mask);

    // std::cout << "showing dilated costmap" << std::endl;
    cv::imshow("Dilated costmap topview", dilated_canvas);
}

void ExtractPlacementOptions(cv::Mat& canvas, cv::Mat& placement_canvas, cv::Scalar targetColor, cv::Scalar point_color, double height)
{

    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    std::vector<cv::Point> identicalPoints;
    cv::Point2d PlacementPoint;
    bool found = false;

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
                PlacementPoint = p;
                found = true;
                if (p.x >= 0 && p.y >= 0 && p.x < placement_canvas.cols && p.y < placement_canvas.rows)
                    placement_canvas.at<cv::Vec3b>(p) = cv::Vec3b(targetColor[0], targetColor[1], targetColor[2]);
            }
        }
    }

    if (!found)
>>>>>>> b07be61 (Dilating per colour basis (not finished))
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

/**
 * @brief usage, print how the executable should be used and explain the input
 */
void usage()
{
    std::cout << "Usage: ed_empty_spot_designator RGBD_TOPIC" << std::endl
              << "RGBD_TOPIC topic on which the rgbd image is published, example /hero/head_rgbd_sensor/rgbd" << std::endl;
}


/**
 * @brief main executable to visualise the empty spot finder of live images.
 * @param argc:
 * @return
 */
int main (int argc, char **argv)
{
    if (argc != 2)
    {
        usage();
        return 0;
    }

    ros::init(argc, argv, "empty_spot_visualizer");

    std::string topic = argv[1];
    std::cout << "Using topic: " << topic << std::endl;

    rgbd::ImageBuffer image_buffer;
    image_buffer.initialize(topic, "base_link");

    pcl::PCDWriter writer;

    PlaceAreaFinder place_area_finder;

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
        
        if (!place_area_finder.findArea(image, sensor_pose, place_pose))
        {
            std::cout << "no place area found" << std::endl;
        }
        std::cout << place_pose << std::endl;

        cv::Mat canvas;
        place_area_finder.getCanvas(canvas);
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

<<<<<<< HEAD
        geo::Pose3D sensor_pose_canvas = sensor_pose;
        sensor_pose_canvas.t.z = sensor_pose.t.z - place_pose.t.z;
        drawFieldOfView(canvas, sensor_pose_canvas, image);
=======
        // Not table cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr notTable_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        notTable_cloud->width       = backup_cloud->width;
        notTable_cloud->height      = backup_cloud->height;
        notTable_cloud->is_dense    = false;
        notTable_cloud->points.resize(notTable_cloud->width*notTable_cloud->height);
        for (int nIndex = 0; nIndex < backup_cloud->points.size (); nIndex++)
        {
        auto lower  = z - backup_cloud->points[nIndex].z;
        auto upper  = height - backup_cloud->points[nIndex].z;
        auto lambda = upper / lower;
        auto dx     = x - backup_cloud->points[nIndex].x;
        auto dy     = y - backup_cloud->points[nIndex].y;       
        notTable_cloud->points[nIndex].z = height;
        notTable_cloud->points[nIndex].x = backup_cloud->points[nIndex].x + lambda * dx;
        notTable_cloud->points[nIndex].y = backup_cloud->points[nIndex].y + lambda * dy;
        }

        
        // std::cout << "creating costmap" << std::endl;
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        // cv::Mat dilated_canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat closed_canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat placement_canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Scalar table_color(0, 255, 0);
        cv::Scalar occupied_color(0, 0, 255);
        cv::Scalar occluded_color(255,0,0);
        cv::Scalar radius_color(100,0,100);
        cv::Scalar placement_color(100, 255, 100);
        cv::Scalar point_color(255,0,0);

        // Object placement margins
        float object_diameter = 0.10;
        float error_margin = 0.02;
        float length = object_diameter + error_margin; 
        float placement_margin = 2*0.02 + length;
        
        std::cout << "creating costmap" << std::endl;

        // Add table plane to costmap
        createCostmap(plane_cloud, canvas, table_color);

        // Adding boundaries with morphological operations

            // // Fill missing data gaps inside the table sheet cluster
            // CloseCanvas(canvas, closed_canvas, placement_margin);

            // // Decrease the size of the table plane to accomodate for the placement radius while taking dilation into account
            // AlterPlane(closed_canvas, closed_canvas, placement_margin);
                
            // // Add objects to costmap
            // createObjectCostmap(object_cloud, closed_canvas, occupied_color);

            // // Add occluded space to costmap
            // createOccludedCostmap(occluded_cloud, closed_canvas, occluded_color); 

            // // HERO preferred radius
            // createRadiusCostmap(closed_canvas, radius_color, placement_margin);

            // // Dilate the costmap and create a new canvas
            // dilateCostmap(closed_canvas, dilated_canvas, placement_margin);

            // // Extract the placement options and choose a placement solution
            // ExtractPlacementOptions(dilated_canvas, placement_canvas, table_color, point_color, height);

        // Adding boundaries with additional PCL data

            // Add objects to costmap
            createObjectCostmap(object_cloud, canvas, occupied_color);

            // Add occluded space to costmap
            createOccludedCostmap(occluded_cloud, canvas, occluded_color); 

            // // Add not_Table to define the table edge                  
            // createNotTableCostmap(notTable_cloud, canvas, occupied_color);             

            // // FOV left
            // createFOVLCostmap(canvas, occluded_color, transform(0,3), transform(1,3), fx);

            // // FOV right
            // createFOVRCostmap(canvas, occluded_color, transform(0,3), transform(1,3));

            // // FOV down
            // createFOVHCostmap(canvas, occluded_color, transform(0,3), transform(1,3), transform(2,3), height);

            // // HERO preferred radius
            // createRadiusCostmap(canvas, radius_color, placement_margin);

            // Dilate the costmap and create a new canvas
            dilateCostmap(canvas, placement_margin);

            // // Extract the placement options and choose a placement solution
            // ExtractPlacementOptions(dilated_canvas, placement_canvas, table_color, point_color, height);            
>>>>>>> b07be61 (Dilating per colour basis (not finished))

        // Show the different canvasses
        
        // std::cout << "showing costmap" << std::endl;
        cv::imshow("Costmap topview", canvas);

        // // std::cout << "showing costmap" << std::endl;
        // cv::imshow("closed canvas topview", closed_canvas);
                
<<<<<<< HEAD
        // std::cout << "showing dilated costmap" << std::endl;
        //cv::imshow("Dilated costmap topview", dilated_canvas);

        // std::cout << "showing placement costmap" << std::endl;
        //cv::imshow("Placement options costmap topview", placement_canvas);
=======


        // // std::cout << "showing placement costmap" << std::endl;
        // cv::imshow("Placement options costmap topview", placement_canvas);
>>>>>>> b07be61 (Dilating per colour basis (not finished))

        // Show RGB snapshot
        cv::Mat rgbcanvas = image->getRGBImage();
        cv::imshow("RGB", rgbcanvas);

        char key = cv::waitKey(30);

        if (key == 'q')
        {
            break;
        }

    }
    cv::destroyAllWindows();
    return 0;
}

#include <iostream>
#include <string>

#include <ed/kinect/image_buffer.h>

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <rgbd/image.h>

//pcl library
#include <pcl/io/pcd_io.h>


double resolution = 0.005;
cv::Point2d canvas_center;

void imageToCloud(const rgbd::Image& image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
            double d = image.getDepthImage().at<float>(i,j);

            cloud->at(j,i).x = (-half_width+j) * d / fx;
            cloud->at(j,i).y = (-half_height+i) * d / fy;
            cloud->at(j,i).z = d;
        }
    }
}

cv::Point2d worldToCanvas(double x, double y)
{
    return cv::Point2d(-y / resolution, -x / resolution) + canvas_center;
}

void createCostmap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat& canvas, cv::Scalar color)
{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (int nIndex = 0; nIndex < cloud->points.size (); nIndex++)
    {
        double x = cloud->points[nIndex].z;
        double y = -cloud->points[nIndex].x;
        //double z = -cloud->points[nIndex].y;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
    }
}

void dilateCostmap(cv::Mat& canvas)
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                             cv::Size( 11, 11),
                                             cv::Point(5, 5) );
    cv::dilate(canvas, canvas, element );
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

    ImageBuffer image_buffer;
    image_buffer.initialize(topic, "base_link");

    while(ros::ok())
    {
        rgbd::ImageConstPtr image;
        geo::Pose3D sensor_pose;

        if (!image_buffer.waitForRecentImage(image, sensor_pose, 2.0))
        {
            std::cerr << "No image received, will try again." << std::endl;
            continue;
        }
        std::cout << "converting image to cloud" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        imageToCloud(*image, cloud);

        std::cout << "creating costmap" << std::endl;
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::Scalar table_color(0, 255, 0);
        cv::Scalar occupied_color(0, 0, 255);

        //createCostmap(occupied_cloud, canvas, occupied_color);
        createCostmap(cloud, canvas, table_color);

        std::cout << "showing costmap" << std::endl;
        cv::imshow("Laser Vizualization", canvas);

        // show snapshot
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

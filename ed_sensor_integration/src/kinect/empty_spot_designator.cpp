#include <iostream>
#include <string>

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

//pcl library
#include <pcl/io/pcd_io.h>


double resolution = 0.005;
cv::Point2d canvas_center;

void readPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename)
{
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read (filename, *cloud); //
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


void usage()
{
    std::cout << "USAGE: empty_spot_designator [pointcloud.pcd] [table_pointcloud.pcd]" << std::endl;
}

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 0;
    }

    std::cout << "finding empty spot" << std::endl;

    std::cout << "Loading pcd file" << std::endl;

    std::string filename = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    readPCD(cloud, filename);

    // read occupied data
    std::string occupied_filename = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    readPCD(occupied_cloud, occupied_filename);

    std::cout << "PointCloud representing the planar component: " << cloud->width * cloud->height << " data points." << std::endl;


    std::cout << "creating costmap" << std::endl;
    cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));
    cv::Scalar table_color(0, 255, 0);
    cv::Scalar occupied_color(0, 0, 255);

    createCostmap(occupied_cloud, canvas, occupied_color);
    //createCostmap(cloud, canvas, table_color);

    std::cout << "showing costmap" << std::endl;
    cv::imshow("Laser Vizualization", canvas);
    cv::waitKey();


    return 0;
}

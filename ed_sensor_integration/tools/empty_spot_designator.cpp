#include <iostream>
#include <string>

#include "ed/kinect/image_buffer.h"

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
        
        place_area_finder.findArea(image, sensor_pose, place_pose);

        cv::Mat canvas;
        place_area_finder.getCanvas(canvas);
        // Show the different canvasses
        
        // std::cout << "showing costmap" << std::endl;
        cv::imshow("Costmap topview", canvas);

        // // std::cout << "showing costmap" << std::endl;
        // cv::imshow("closed canvas topview", closed_canvas);
                
        // std::cout << "showing dilated costmap" << std::endl;
        //cv::imshow("Dilated costmap topview", dilated_canvas);

        // std::cout << "showing placement costmap" << std::endl;
        //cv::imshow("Placement options costmap topview", placement_canvas);

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

#include <iostream>
#include <string>

#include "ed/kinect/image_buffer.h"

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <rgbd/image.h>

//pcl library # TODO remove the unused ones #TODO find out which ones are unused
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

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

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/convex_hull.h>

#include "ed_sensor_integration/sac_model_horizontal_plane.h"


double resolution = 0.005;
cv::Point2d canvas_center;

void imageToCloud(const rgbd::Image& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
}

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

    std::cout << Transform << std::endl;
    return Transform;
}

/**
 * @brief FilterPlane fit a plane through a pointcloud, filter the points which lie in this plane and return the height of the plane #TODO separation of concerns.
 * @param cloud: pointcloud to be filtered.
 * @param out: pointcloud with all points that lie within the plane
 * @return height (z coordinate) of the fitted plane.
 */
float FilterPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{
    std::vector<int> indices;
    float threshold = 0.03;

    std::cout << "starting ransac" << std::endl;
    // Create SAC model
    pcl::SampleConsensusModelHorizontalPlane<pcl::PointXYZRGB>::Ptr plane (new pcl::SampleConsensusModelHorizontalPlane<pcl::PointXYZRGB>(cloud));
    std::cout << "created plane object" << std::endl;
    // Create SAC method
    pcl::RandomSampleConsensus<pcl::PointXYZRGB>::Ptr sac (new pcl::RandomSampleConsensus<pcl::PointXYZRGB> (plane, threshold));
    std::cout << "created ransac object" << std::endl;
    sac->setMaxIterations(10000);
    sac->setProbability(0.99);

    // Fit model
    sac->computeModel();

    // Get inliers
    std::vector<int> inliers;
    sac->getInliers(inliers);

    // Get the model coefficients
    Eigen::VectorXf coeff;
    sac->getModelCoefficients (coeff);
    std::cout << "ransac complete" << std::endl;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, (coeff[3]-0.01))));
    *out = *cloud;
    //filter out everything below plane
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (out);
    condrem.setKeepOrganized(true);

    condrem.filter (*out);
    (*out).is_dense = false;
    pcl::removeNaNFromPointCloud(*out, *out, indices);

    return(coeff[3]);
}

/**
 * @brief Segment segment the pointcloud and return the cluster closest to the camera
 * @param cloud: pointcloud to be segmented, this function will change the pointcloud to only include the segmented cluster.
 * @param x: coordinate of the camera
 * @param y: coordinate of the camera
 * @param z: coordinate of the camera
 * @param [out] tableHeight[m]: estimated height of the table based on the cluster.
 */
void Segment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y, float z)
{
    std::cout << "starting segmentation" << std::endl;
    std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; //using euclidian cluster extraction
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((*cloud).size()/100); //#TODO magic number
    ec.setMaxClusterSize ((*cloud).size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::cout << "obtained " << cluster_indices.size() << " cluster indices" <<std::endl;

    //find closest cluster
    pcl::PointIndices closest_cluster;
    float mindist_sq = INFINITY;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it !=
         cluster_indices.end (); ++it) //iterate through all clusters
    {
        //construct cluster
        float sumx = 0, sumy = 0, sumz = 0, dist_sq = 0;
        int n = 0;
        for (const auto& idx : it->indices)
        {
            sumx += (*cloud)[idx].x;
            sumy += (*cloud)[idx].y;
            sumz += (*cloud)[idx].z;
            n++;
        }

        //find distance from camera to the middle of the cluster
        dist_sq = pow((sumx/n-x),2) + pow((sumy/n-y),2) + pow((sumz/n-z),2);
        std::cout << "distance is " << sqrt(dist_sq) << std::endl;
        if (dist_sq < mindist_sq) //check if closest so far
        {
            std::cout << "updating closest cluster" << std::endl;
            mindist_sq = dist_sq;
            closest_cluster = *it;
        }
    }

    //construct cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (uint i : closest_cluster.indices)
        cloud_out->push_back( (*cloud)[i] ); //*

    float height = FilterPlane(cloud_out, cloud_out);

    std::cout << "writing closest cluster" << std::endl;
    *cloud = *cloud_out;
}

cv::Point2d worldToCanvas(double x, double y)
{
    return cv::Point2d(-y / resolution, -x / resolution) + canvas_center;
}

void createCostmap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat& canvas, cv::Scalar color)
{
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    for (int nIndex = 0; nIndex < cloud->points.size (); nIndex++)
    {
        double x = cloud->points[nIndex].x;
        double y = -cloud->points[nIndex].y;

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

    pcl::PCDWriter writer;

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        imageToCloud(*image, cloud);

        // transform to base link frame
        Eigen::Matrix4f transform = geolibToEigen(sensor_pose);
        pcl::transformPointCloud(*cloud, *cloud, transform);

        // keep track of the indices in the original image
        std::vector<int> indices;

        // Filter out floor
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.1)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud);
        (*cloud).is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        Segment(cloud, 0.0, 0.0, 0.0);

        std::cout << "creating costmap" << std::endl;
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::Scalar table_color(0, 255, 0);
        cv::Scalar occupied_color(0, 0, 255);

        //createCostmap(occupied_cloud, canvas, occupied_color);
        createCostmap(cloud, canvas, table_color);

        std::cout << "showing costmap" << std::endl;
        cv::imshow("Costmap topview", canvas);

        // show snapshot
        cv::Mat rgbcanvas = image->getRGBImage();
        cv::imshow("RGB", rgbcanvas);

        char key = cv::waitKey(30);

        if (key == 32)
        {
            writer.write<pcl::PointXYZRGB> ("segmented_cloud.pcd", *cloud, false);
            std::cout << "wrote current cloud to 'segmented_cloud.pcd'." << std::endl;
        }
        else if (key == 'q')
        {
            break;
        }

    }
    cv::destroyAllWindows();
    return 0;
}

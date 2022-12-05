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


double resolution = 0.005;
cv::Point2d canvas_center;

void imageToCloud(const rgbd::Image& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FOVL (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = image.getDepthImage().cols;
    cloud->height = image.getDepthImage().rows;
    // FOVL->width = image.getDepthImage(0).col;
    // FOVL->height = image.getDepthImage().rows;
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

    // std::cout << Transform << std::endl;
    return Transform;
}

// /**
//  * @brief FilterPlane fit a plane through a pointcloud, filter the points which lie in this plane and return the height of the plane #TODO separation of concerns.
//  * @param cloud_backup: pointcloud to be filtered.
//  * @param out: pointcloud with all points that lie within the plane
//  * @return height (z coordinate) of the fitted plane.
//  */
// float FilterPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_backup, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
// {
//     std::vector<int> indices;
//     float threshold = 0.03;

//     std::cout << "starting ransac" << std::endl;
//     // Create SAC model
//     pcl::SampleConsensusModelHorizontalPlane<pcl::PointXYZRGB>::Ptr plane (new pcl::SampleConsensusModelHorizontalPlane<pcl::PointXYZRGB>(cloud_backup));
//     std::cout << "created plane object" << std::endl;
//     // Create SAC method
//     pcl::RandomSampleConsensus<pcl::PointXYZRGB>::Ptr sac (new pcl::RandomSampleConsensus<pcl::PointXYZRGB> (plane, threshold));
//     std::cout << "created ransac object" << std::endl;
//     sac->setMaxIterations(10000);
//     sac->setProbability(0.99);

//     // Fit model
//     sac->computeModel();

//     // Get inliers
//     std::vector<int> inliers;
//     sac->getInliers(inliers);

//     // Get the model coefficients
//     Eigen::VectorXf coeff;
//     sac->getModelCoefficients (coeff);
//     std::cout << "ransac complete" << std::endl;

//     pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, (coeff[3]-0.01))));
//     *out = *cloud_backup;

//     //filter out everything below plane
//     pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
//     condrem.setCondition (range_cond);
//     condrem.setInputCloud (out);
//     condrem.setKeepOrganized(true);

//     condrem.filter (*out);
//     (*out).is_dense = false;
//     pcl::removeNaNFromPointCloud(*out, *out, indices);

//     return(coeff[3]);
    

//  }

// /**
//  * @brief Segment the pointcloud and return the cluster closest to the camera
//  * @param cloud: pointcloud to be segmented, this function will change the pointcloud to only include the segmented cluster.
//  * @param x: coordinate of the camera
//  * @param y: coordinate of the camera
//  * @param z: coordinate of the camera
//  * @param [out] height [m]: estimated height of the table based on the cluster.
//  */
// void Segment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y, float z,pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud)
// {
//     std::cout << "starting segmentation" << std::endl;
//     std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

//     // Creating the KdTree object for the search method of the extraction
//     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//     tree->setInputCloud (cloud);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; //using euclidian cluster extraction
//     ec.setClusterTolerance (0.1);
//     ec.setMinClusterSize ((*cloud).size()/100); //#TODO magic number
//     ec.setMaxClusterSize ((*cloud).size());
//     ec.setSearchMethod (tree);
//     ec.setInputCloud (cloud);
//     ec.extract (cluster_indices);

//     std::cout << "obtained " << cluster_indices.size() << " cluster indices" <<std::endl;

//     //find closest cluster
//     pcl::PointIndices closest_cluster;
//     float mindist_sq = INFINITY;
//     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it !=
//          cluster_indices.end (); ++it) //iterate through all clusters
//     {
//         //construct cluster
//         float sumx = 0, sumy = 0, sumz = 0, dist_sq = 0;
//         int n = 0;
//         for (const auto& idx : it->indices)
//         {
//             sumx += (*cloud)[idx].x;
//             sumy += (*cloud)[idx].y;
//             sumz += (*cloud)[idx].z;
//             n++;
//         }

//         //find distance from camera to the middle of the cluster
//         dist_sq = pow((sumx/n-x),2) + pow((sumy/n-y),2) + pow((sumz/n-z),2);
//         std::cout << "distance is " << sqrt(dist_sq) << std::endl;
//         if (dist_sq < mindist_sq) //check if closest so far
//         {
//             std::cout << "updating closest cluster" << std::endl;
//             mindist_sq = dist_sq;
//             closest_cluster = *it;
//         }
//     }

    // //construct cluster
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (uint i : closest_cluster.indices)
    //     cloud_out->push_back( (*cloud)[i] ); //*

    // // float height = FilterPlane(cloud_out, cloud_out);

    // std::cout << "writing closest cluster" << std::endl;

    // // Filter out objects and put them in seperate cloud
    // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
    // pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, height+0.02)));
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
    // pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, height+0.30)));
    // // build the filter
    // pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    // condrem.setCondition (range_cond);
    // condrem.setInputCloud (cloud);
    // condrem.setKeepOrganized(true);
    // // apply filter
    // condrem.filter (*object_cloud);
    
    // Update cloud
    // *cloud = *cloud_out;

// }


/**
 * @brief SegmentPlane segment the pointcloud and return the cluster closest to the camera
 * @param cloud: pointcloud to be segmented, this function will change the pointcloud to only include the segmented cluster.
 */
void SegmentPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
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

    int i = 0, nr_points = (int) cloud->size ();

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    // std::cout << "PointCloud representing the planar component: " << inliers->indices.size() << " data points."
    //  << "Plane with coefficients: " << *coefficients << std::endl;

    cloud->swap(*cloud_p);
}
// cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));

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

void createFOVLCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y)

{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        for (int nIndex = 0; nIndex < 3000 ; nIndex++)
        {
        float initial_x = x;
        float initial_y = y;
        double y = initial_y + 0.001*nIndex;
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
        for (int i = 0; i < 8; i++)
        {
            for (int nIndex = 0; nIndex < 4000; nIndex++)
            {
            float initial_x = x;
            float initial_y = y;
            double y = initial_y + -2+ 0.001*nIndex;
            double x = initial_x + (z-height)*tan(67.5/(180/M_PI)); // 0.125*i*

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
                canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
}

void createRadiusCostmap(cv::Mat& canvas, cv::Scalar color)
{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        float radius = 0.70;
            for (float phi = 0; phi < 360; phi++)
            {

            double y = radius * sin(phi/(180/M_PI));
            double x = radius * cos(phi/(180/M_PI));

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
                canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
            }
        
}

void dilateCostmap(cv::Mat& canvas)
{
    float resolution = 0.005;
    float radius = 0.10;
    float margin = 0.01;
    float length = radius + margin; 
    float Pixelsize = length / resolution;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::dilate(canvas, canvas, element );
}

double ExtractPlacementOption(cv::Mat& canvas, cv::Scalar targetColor)
{

    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);

    std::vector<cv::Point> identicalPoints;

    for(int row = canvas.rows; row > 0; --row)
    {
        for(int col = canvas.rows; col > 0; --col)
        {
            cv::Vec3b currPixel = canvas.at<cv::Vec3b>(row, col);
            if(currPixel.val[0] == targetColor.val[0] &&
               currPixel.val[1] == targetColor.val[1] &&
               currPixel.val[2] == targetColor.val[2])
            {
                identicalPoints.push_back(cv::Point(col, row));
            }
        }
    }
    

    cv::Point Point = identicalPoints[0];
    double x = (Point.x-canvas_center.x)*-resolution;
    double y = (Point.y-canvas_center.y)*-resolution;



    std::cout << "The point on the canvas that is closest to HERO is:" << std::endl;
    std::cout << Point << std::endl;
    std::cout << "With X-coordinate:" << std::endl;
    std::cout << x << std::endl;
    std::cout << "And Y-coordinate:" << std::endl;
    std::cout << y << std::endl;  


    return x;
    return y;
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
        // std::cout << "converting image to cloud" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        imageToCloud(*image, cloud);

        // transform to base link frame
        Eigen::Matrix4f transform = geolibToEigen(sensor_pose);
        pcl::transformPointCloud(*cloud, *cloud, transform);

        // keep track of the indices in the original image
        std::vector<int> indices;


        // Filter out objects and put them in seperate cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   
        // Get the height of the table for the object detection
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
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

        int i = 0, nr_points = (int) cloud->size ();

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        float height = abs(coefficients->values[3]);

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, height+0.03)));
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, height+0.30)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem2;
        condrem2.setCondition (range_cond2);
        condrem2.setInputCloud (cloud);
        condrem2.setKeepOrganized(true);
        // apply filter
        condrem2.filter (*object_cloud);
        pcl::removeNaNFromPointCloud(*object_cloud, *object_cloud, indices);





        // Create pointcloud with occluded space
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr occluded_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        occluded_cloud->width=object_cloud->width;
        occluded_cloud->height=object_cloud->height;
        occluded_cloud->is_dense = false;
        occluded_cloud->points.resize(occluded_cloud->width*occluded_cloud->height);
        float x = transform(0,3);
        float y = transform(1,3);
        float z = transform(2,3);
        for (int nIndex = 0; nIndex < object_cloud->points.size (); nIndex++)
        {
        auto lower = object_cloud->points[nIndex].z - z;
        auto upper = height - object_cloud->points[nIndex].z;
        auto lambda = upper / lower;
        auto dx = object_cloud->points[nIndex].x - x;
        auto dy = object_cloud->points[nIndex].y- y;

        
        occluded_cloud->points[nIndex].z =  height;
        occluded_cloud->points[nIndex].x = object_cloud->points[nIndex].x + lambda * dx;
        occluded_cloud->points[nIndex].y = object_cloud->points[nIndex].y + lambda * dy;
        }

        // Filter out items above table
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr backup_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond3 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond3->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, height-0.05)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem3;
        condrem3.setCondition (range_cond3);
        condrem3.setInputCloud (cloud);
        condrem3.setKeepOrganized(true);
        // apply filter
        condrem3.filter (*backup_cloud);
        (*backup_cloud).is_dense = false;
        pcl::removeNaNFromPointCloud(*backup_cloud, *backup_cloud, indices);

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


        SegmentPlane(cloud);
  

        
        // std::cout << "creating costmap" << std::endl;
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::Scalar table_color(0, 255, 0);
        cv::Scalar occupied_color(0, 0, 255);
        cv::Scalar occluded_color(255,0,0);
        cv::Scalar radius_color(100,0,100);
        cv::Scalar placement_color(100, 255, 100);
        


        // Add not table to costmap
        createNotTableCostmap(notTable_cloud, canvas, occupied_color); 

        // Add table plane to costmap
        createCostmap(cloud, canvas, table_color);

        // prints on top of costmap(object_cloud, canvas, occupied_color)
        createOccludedCostmap(occluded_cloud, canvas, occluded_color);       

        // prints on top of costmap(object_cloud, canvas, occupied_color)
        createObjectCostmap(object_cloud, canvas, occupied_color);

        // FOV left
        createFOVLCostmap(canvas, occupied_color, transform(0,3), transform(1,3));

        // FOV right
        createFOVRCostmap(canvas, occupied_color, transform(0,3), transform(1,3));

        // FOV down
        createFOVHCostmap(canvas, occupied_color, transform(0,3), transform(1,3), transform(2,3), height);

        // HERO preferred radius
        createRadiusCostmap(canvas, radius_color);

        // Dilate the costmap
        dilateCostmap(canvas);

        ExtractPlacementOption(canvas, placement_color);
    

        // std::cout << "showing costmap" << std::endl;
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

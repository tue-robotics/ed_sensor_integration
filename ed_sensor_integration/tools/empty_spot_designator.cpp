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

double imageToCloud(const rgbd::Image& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr FOVL)
{
    // Fill in the cloud data
    cloud->width = image.getDepthImage().cols;
    cloud->height = image.getDepthImage().rows;
    cloud->is_dense = false;
    cloud->resize (cloud->width * cloud->height);

    FOVL->width = cloud->width;
    FOVL->height = image.getDepthImage().rows;
    FOVL->is_dense = false;
    FOVL->resize (FOVL->width * FOVL->height);

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

            // FOVL->at(j,0).y = cloud->at(j,0).y;
            // FOVL->at(j,0).x = cloud->at(j,0).x;

        }
    }

return image.getCameraModel().fx();
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
 * @return height of the segmented plane
 */
float SegmentPlane (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_plane)
{
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
    pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
    int i = 0, nr_points = (int) cloud_in->size ();

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return -1.0;
    }

    // Extract the inliers to a cloud with a plane
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);

    
    // std::cout << "PointCloud representing the planar component: " << inliers->indices.size() << " data points."
    //   << "Plane with coefficients: " << *coefficients << std::endl;

    // Extract outliers to the main cloud without the table plane
    extract2.setInputCloud(cloud_in2);
    extract2.setIndices(inliers);
    extract2.setNegative(true);
    extract2.filter(*cloud_no_plane);

    return abs(coefficients->values[3]);
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

void createFOVLCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y, double fx)

{
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols);
        for (int nIndex = 0; nIndex < 3000 ; nIndex++)
        {
        // double d = image.getDepthImage().at<float>(i,j);
        // double fx = image.getCameraModel().fx();
        float dpix = canvas.cols;
        float initial_x = x;
        float initial_y = y;
        double y = initial_y + (0.001)*nIndex;
        double x = initial_x + 0.001*nIndex*tan(60/(180/M_PI));
        // double x = y * (1/fx) *dpix;
        // std::cout << x << std::endl;

        cv::Point2d p = worldToCanvas(x, y);
        if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
            canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);
        }


        // for (int nIndex = 0; nIndex < 500 ; nIndex++)
        // {
        // float dpix = nIndex/2;
        // double y = 0.005*nIndex;
        // double x = y * (1/fx) * dpix;
        // std::cout << x << std::endl;
        // cv::Point2d p = worldToCanvas(x, y);
        // if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
        //     canvas.at<cv::Vec3b>(p) = cv::Vec3b(color[0], color[1], color[2]);

        // }
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

void createFOVHCostmap(cv::Mat& canvas, cv::Scalar color, float x, float y, float z, float height) // Replace with morphology
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

void dilateCostmap(cv::Mat& canvas, cv::Mat& dilated_canvas, float placement_margin)
{
    float Pixelsize = placement_margin / resolution;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( Pixelsize, Pixelsize),
                                             cv::Point(-1, -1) );
    cv::dilate(canvas, dilated_canvas, element );

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
        return;

    double y = (PlacementPoint.x-canvas_center.x)*-resolution;
    double x = (PlacementPoint.y-canvas_center.y)*-resolution;

    double margin = 0.02;

    std::cout << "The selected point for placement in (x,y,z) coordinates is:" << std::endl;
    std::cout << "(" << x << ", " << y << ", " << height+margin << ")" << std::endl;
    std::cout << "Which is " << sqrt(pow(x,2)+pow(y,2)) << " cm away from HERO" << std::endl;
    placement_canvas.at<cv::Vec3b>(PlacementPoint) = cv::Vec3b(point_color[0], point_color[1], point_color[2]); 

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr FOVL (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr FOVR (new pcl::PointCloud<pcl::PointXYZRGB>);
        double fx = imageToCloud(*image, cloud, FOVL);

        // transform to base link frame
        Eigen::Matrix4f transform = geolibToEigen(sensor_pose);
        pcl::transformPointCloud(*cloud, *cloud, transform);

        // keep track of the indices in the original image
        std::vector<int> indices;


        // Filter out floor
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr floorless_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.1)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*floorless_cloud);
        (*floorless_cloud).is_dense = false;
        pcl::removeNaNFromPointCloud(*floorless_cloud, *floorless_cloud, indices);

        std::cout << "SegmentPlane" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeless_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Segment the table plane and return a cloud with the plane and a cloud where the plane is removed
        float height = SegmentPlane(floorless_cloud, floorless_cloud, plane_cloud, planeless_cloud);
        std::cout << "Found plane height " << height << std::endl;


        // Filter out objects and put them in seperate cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, height)));
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, height+0.30)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem2;
        condrem2.setCondition (range_cond2);
        condrem2.setInputCloud (planeless_cloud);
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

        // Filter out items below table to create not table cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr backup_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond3 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond3->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, height-0.02)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem3;
        condrem3.setCondition (range_cond3);
        condrem3.setInputCloud (floorless_cloud);
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

        
        // std::cout << "creating costmap" << std::endl;
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat dilated_canvas(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
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
                
        // Add objects to costmap
        createObjectCostmap(object_cloud, canvas, occupied_color);

        // Add occluded space to costmap
        createOccludedCostmap(occluded_cloud, canvas, occluded_color); 


        ///// Replace the commented steps with image processing technniques instead


        // // Add not table to costmap 
        // createNotTableCostmap(notTable_cloud, canvas, occupied_color); 

        // // FOV left
        // createFOVLCostmap(canvas, occluded_color, transform(0,3), transform(1,3), fx);

        // // FOV right
        // createFOVRCostmap(canvas, occluded_color, transform(0,3), transform(1,3));

        // // FOV down
        // createFOVHCostmap(canvas, occluded_color, transform(0,3), transform(1,3), transform(2,3), height);

        // HERO preferred radius
        createRadiusCostmap(canvas, radius_color, placement_margin);

        // Dilate the costmap and create a new canvas
        dilateCostmap(canvas, dilated_canvas, placement_margin);

        // Extract the placement options and choose a placement solution
        ExtractPlacementOptions(dilated_canvas, placement_canvas, table_color, point_color, height);

        std::cout << "showing costmap" << std::endl;
        cv::imshow("Costmap topview", canvas);

        std::cout << "showing placement costmap" << std::endl;
        cv::imshow("Placement options costmap topview", placement_canvas);

        std::cout << "showing dilated costmap" << std::endl;
        cv::imshow("Dilated costmap topview", dilated_canvas);

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

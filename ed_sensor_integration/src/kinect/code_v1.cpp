#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>

#include <ed/io/json_reader.h>
#include <ed/serialization/serialization.h>
//#include "ed_sensor_integration/sac_model_circle.h"

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <fstream>
#include <boost/filesystem/convenience.hpp>

Eigen::Matrix4f ReadJson(std::string pcd_filename, float *xout, float *yout, float *zout) {

    std::string json_filename = "2022-04-22-12-04-46.json";
    //std::string json_filename = boost::filesystem::change_extension(pcd_filename, ".json").string();
    // read json metadata
    tue::config::DataPointer meta_data;

    try
    {
        meta_data = tue::config::fromFile(json_filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << json_filename << "'.\n\n" << e.what() << std::endl;
        //return 0;
    }

    tue::config::Reader r(meta_data);
    // Read sensor pose
    geo::Pose3D sensor_pose;
    if (!ed::deserialize(r, "sensor_pose", sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        //return 0;
    }
    // convert from geolib coordinates to ros coordinates #TODO remove geolib coordinates for camera pose
    sensor_pose.R = sensor_pose.R * geo::Mat3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    float x = sensor_pose.t.x;
    float y = sensor_pose.t.y;
    float z = sensor_pose.t.z;
    float xx = sensor_pose.R.xx;
    float xy = sensor_pose.R.xy;
    float xz = sensor_pose.R.xz;
    float yx = sensor_pose.R.yx;
    float yy = sensor_pose.R.yy;
    float yz = sensor_pose.R.yz;
    float zx = sensor_pose.R.zx;
    float zy = sensor_pose.R.zy;
    float zz = sensor_pose.R.zz;

    *xout = x;
    *yout = y;
    *zout = z;

    //float qx, qy, qz, qw;

    //const float n = 2.0f/(qx*qx+qy*qy+qz*qz+qw*qw);
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();/* {
        {1.0f - n*qy*qy - n*qz*qz, n*qx*qy - n*qz*qw, n*qx*qz + n*qy*qw, x},
        {n*qx*qy + n*qz*qw, 1.0f - n*qx*qx - n*qz*qz, n*qy*qz - n*qx*qw, y},
        {n*qx*qz - n*qy*qw, n*qy*qz + n*qx*qw, 1.0f - n*qx*qx - n*qy*qy, z},
        {0.0f, 0.0f, 0.0f, 1.0f}}; */

    Transform(0,0) = xx;//1.0f - n*qy*qy - n*qz*qz;
    Transform(0,1) = xy;//n*qx*qy - n*qz*qw;
    Transform(0,2) = xz;//n*qx*qz + n*qy*qw;
    Transform(0,3) = x;
    Transform(1,0) = yx;//n*qx*qy + n*qz*qw;
    Transform(1,1) = yy;//1.0f - n*qx*qx - n*qz*qz;
    Transform(1,2) = yz;//n*qy*qz - n*qx*qw;
    Transform(1,3) = y;
    Transform(2,0) = zx;//n*qx*qz - n*qy*qw;
    Transform(2,1) = zy;//n*qy*qz + n*qx*qw;
    Transform(2,2) = zz;//1.0f - n*qx*qx - n*qy*qy;
    Transform(2,3) = z;

    std::cout << Transform << std::endl;
    return Transform;
}

float FilterPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out) {

    //*out = *cloud; return(-1); //activate to bypass plane fitting and height estimation

    std::vector<int> indices;
    float threshold = 0.03;

    std::cout << "starting ransac" << std::endl;
    // Create SAC model
    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr plane (new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>(cloud));
    plane->setAxis (Eigen::Vector3f(1,0,0));
    plane->setEpsAngle(15*0.0174532925); //*0.0174532925 to radians
    std::cout << "created plane object" << std::endl;
    // Create SAC method
    //pcl::SACSegmentation<pcl::PointXYZ> segplane;
    pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr segplane (new pcl::RandomSampleConsensus<pcl::PointXYZ> (plane, threshold));
    std::cout << "created ransac object" << std::endl;
    segplane->setMaxIterations(10000);
    segplane->setProbability(0.99);

    // Fit model
    segplane->computeModel();

    // Get inliers
    std::vector<int> inliers;
    segplane->getInliers(inliers);

    // Get the model coefficients
    Eigen::VectorXf coeff;
    segplane->getModelCoefficients (coeff);
    std::cout << "ransac complete" << std::endl;

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, (coeff[3]-0.01))));
    //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, (coeff[3]+0.01))));
    *out = *cloud;
    //filter out everything below plane
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (out);
    condrem.setKeepOrganized(true);

    condrem.filter (*out);
    (*out).is_dense = false;
    pcl::removeNaNFromPointCloud(*out, *out, indices);

    return(coeff[3]);

}

int
main (int argc, char **argv)
{
  std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > inputs;
//  float x, y, z;//used to store camera position
//      pcl::transformPointCloud (*inputs[0], *inputs[0], ReadJson(argv[1], &x, &y, &z));

  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("2022-04-22-12-04-46.pcd", *cloud_blob); //

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 0.25cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.0025f, 0.0025f, 0.0025f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ> model (cloud_filtered);
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setAxis (Eigen::Vector3f(1,0,0));
  seg.setEpsAngle(15*0.0174532925); //*0.0174532925 to radians

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_p);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_p);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : it->indices)
      cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
  }

  //Removing outliers using a StatisticalOutlierRemoval filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader2;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("table_scene_lms400_plane_1.pcd", *cloud2);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud2 << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud2);
  sor2.setMeanK (50);
  sor2.setStddevMulThresh (1.0);
  sor2.filter (*cloud_filtered2);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered2 << std::endl;

  pcl::PCDWriter writer2;
  writer2.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered2, false);

  sor2.setNegative (true);
  sor2.filter (*cloud_filtered2);
  writer2.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered2, false);

  FilterPlane;

//  pcl::SampleConsensusModelCircle<pcl::PointXYZ>::Ptr circle (new pcl::SampleConsensusModelCircle<pcl::PointXYZ>(cloud_ptr));

//  // Create SAC method
//  pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr saccirc (new pcl::RandomSampleConsensus<pcl::PointXYZ> (circle, threshold));
//  saccirc->setMaxIterations(10000);
//  saccirc->setProbability(1);

//  // Fit model
//  saccirc->computeModel();

//  // Get inliers
//  std::vector<int> inliers3;
//  saccirc->getInliers(inliers3);

//  // Get the model coefficients
//  Eigen::VectorXf coeff3;
//  saccirc->getModelCoefficients (coeff3);

//  float min_inliers = 0.0;
//  Eigen::VectorXf output;

//  std::cout << "Circle model, " << static_cast<float>(inliers3.size())/static_cast<float>(cloud_p.size()) << std::endl;

  return (0);

}

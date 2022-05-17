#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <ed/io/json_reader.h>
#include <ed/serialization/serialization.h>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <fstream>
#include <boost/filesystem/convenience.hpp>

Eigen::Matrix4f ReadJson(std::string pcd_filename, float *xout, float *yout, float *zout) {

    std::string json_filename = boost::filesystem::change_extension(pcd_filename, ".json").string();
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

float FilterPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out) {

    //*out = *cloud; return(-1); //activate to bypass plane fitting and height estimation

    std::vector<int> indices;
    float threshold = 0.03;

    std::cout << "starting ransac" << std::endl;
    // Create SAC model
    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZRGB>::Ptr plane (new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZRGB>(cloud));
    plane->setAxis (Eigen::Vector3f(1,0,0));
    plane->setEpsAngle(15*0.0174532925); //*0.0174532925 to radians
    std::cout << "created plane object" << std::endl;
    // Create SAC method
    pcl::SACSegmentation<pcl::PointXYZ> segplane;
    //pcl::SampleConsensus<pcl::PointXYZRGB>::Ptr sac (new pcl::SampleConsensus<pcl::PointXYZRGB> (plane, threshold));
    std::cout << "created ransac object" << std::endl;
    segplane.setMaxIterations(10000);
    segplane.setProbability(0.99);

    // Fit model
    //segplane.computeModel();

    // Get inliers
    std::vector<int> inliers;
    //segplane.getInliers(inliers);

    // Get the model coefficients
    Eigen::VectorXf coeff;
    //segplane.getModelCoefficients (coeff);
    std::cout << "ransac complete" << std::endl;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, (coeff[3]-0.01))));
    //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, (coeff[3]+0.01))));
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

int
main ()
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("2022-04-05-13-28-28.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
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
  model.setAxis (Eigen::Vector3f(1,0,0));
  model.setEpsAngle(15*0.0174532925); //*0.0174532925 to radians

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

  return (0);


}

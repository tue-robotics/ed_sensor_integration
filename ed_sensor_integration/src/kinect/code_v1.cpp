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

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/plugins/obstale_layer.cpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

#include <fstream>
#include <boost/filesystem/convenience.hpp>

//typedef pcl::PointXYZ PointType;

//// --------------------
//// -----Parameters-----
//// --------------------
//float angular_resolution = 0.5f;
//float support_size = 0.2f;
//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//bool setUnseenToMaxRange = false;
//bool rotation_invariant = true;

//// --------------
//// -----Help-----
//// --------------
//void
//printUsage (const char* progName)
//{
//  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
//            << "Options:\n"
//            << "-------------------------------------------\n"
//            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
//            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
//            << "-m           Treat all unseen points to max range\n"
//            << "-s <float>   support size for the interest points (diameter of the used sphere - "
//                                                                  "default "<<support_size<<")\n"
//            << "-o <0/1>     switch rotational invariant version of the feature on/off"
//            <<               " (default "<< (int)rotation_invariant<<")\n"
//            << "-h           this help\n"
//            << "\n\n";
//}

//void
//setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
//{
//  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
//  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
//  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
//  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
//                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
//                            up_vector[0], up_vector[1], up_vector[2]);
//}

Eigen::Matrix4f ReadJson(std::string pcd_filename, float *xout, float *yout, float *zout) {

    std::string json_filename = "2022-04-22-11-44-48.json";
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

//float FilterPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out) {

//    //*out = *cloud; return(-1); //activate to bypass plane fitting and height estimation

//    std::vector<int> indices;
//    float threshold = 0.03;

//    std::cout << "starting ransac" << std::endl;
//    // Create SAC model
//    pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr plane (new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>(cloud));
//    plane->setAxis (Eigen::Vector3f(1,0,0));
//    plane->setEpsAngle(15*0.0174532925); //*0.0174532925 to radians
//    std::cout << "created plane object" << std::endl;
//    // Create SAC method
//    //pcl::SACSegmentation<pcl::PointXYZ> segplane;
//    pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr segplane (new pcl::RandomSampleConsensus<pcl::PointXYZ> (plane, threshold));
//    std::cout << "created ransac object" << std::endl;
//    segplane->setMaxIterations(10000);
//    segplane->setProbability(0.99);

//    // Fit model
//    segplane->computeModel();

//    // Get inliers
//    std::vector<int> inliers;
//    segplane->getInliers(inliers);

//    // Get the model coefficients
//    Eigen::VectorXf coeff;
//    segplane->getModelCoefficients (coeff);
//    std::cout << "ransac complete" << std::endl;

//    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
//    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, (coeff[3]-0.01))));
//    //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, (coeff[3]+0.01))));
//    *out = *cloud;
//    //filter out everything below plane
//    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//    condrem.setCondition (range_cond);
//    condrem.setInputCloud (out);
//    condrem.setKeepOrganized(true);

//    condrem.filter (*out);
//    (*out).is_dense = false;
//    pcl::removeNaNFromPointCloud(*out, *out, indices);

//    return(coeff[3]);

//}

//using namespace cv;
//using namespace std;
//int thresh = 100;
//RNG rng(12345);
//void thresh_callback(int, void* );

//Mat src, src_gray;
//Mat dst, detected_edges;

//int lowThreshold = 0;
//const int max_lowThreshold = 100;
//const int ratio = 3;
//const int kernel_size = 3;
//const char* window_name = "Edge Map";
//static void CannyThreshold(int, void*)
//{
//    blur( src_gray, detected_edges, Size(3,3) );
//    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
//    dst = Scalar::all(0);
//    src.copyTo( dst, detected_edges);
//    imshow( window_name, dst );
//}

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
  reader.read ("2022-04-22-11-44-48.pcd", *cloud_blob); //

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

//  // Creating the KdTree object for the search method of the extraction
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud_p);

//  std::vector<pcl::PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//  ec.setClusterTolerance (0.02); // 2cm
//  ec.setMinClusterSize (100);
//  ec.setMaxClusterSize (100000);
//  ec.setSearchMethod (tree);
//  ec.setInputCloud (cloud_p);
//  ec.extract (cluster_indices);

//  int j = 0;
//  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//  {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//      for (const auto& idx : it->indices)
//      cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
//      cloud_cluster->width = cloud_cluster->size ();
//      cloud_cluster->height = 1;
//      cloud_cluster->is_dense = true;

//      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
//      std::stringstream ss;
//      ss << "cloud_cluster_" << j << ".pcd";
//      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//      j++;
//  }

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
  sor2.setMeanK (1);
  sor2.setStddevMulThresh (0.001);
  sor2.filter (*cloud_filtered2);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered2 << std::endl;

  pcl::PCDWriter writer2;
  writer2.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered2, false);

  sor2.setNegative (true);
  sor2.filter (*cloud_filtered2);
  writer2.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered2, false);

  //FilterPlane;

  //Line detectors

//  CommandLineParser parser( argc, argv, "{@input | cluster.png | input image}" );
//  Mat src = imread( samples::findFile( parser.get<String>( "@input" ) ) );
//  if( src.empty() )
//  {
//      cout << "Could not open or find the image!\n" << endl;
//      cout << "usage: " << argv[0] << " <Input image>" << endl;
//      return -1;
//  }
//  cvtColor( src, src_gray, COLOR_BGR2GRAY );
//  blur( src_gray, src_gray, Size(3,3) );
//  const char* source_window = "Source";
//  namedWindow( source_window );
//  imshow( source_window, src );
//  const int max_thresh = 255;
//  createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
//  thresh_callback( 0, 0 );
//  waitKey();
//  return 0;
//}

//void thresh_callback(int, void* )
//{
//  Mat canny_output;
//  Canny( src_gray, canny_output, thresh, thresh*2 );
//  vector<vector<Point> > contours;
//  findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
//  vector<vector<Point> > contours_poly( contours.size() );
//  vector<Rect> boundRect( contours.size() );
//  vector<Point2f>centers( contours.size() );
//  vector<float>radius( contours.size() );
//  for( size_t i = 0; i < contours.size(); i++ )
//  {
//      approxPolyDP( contours[i], contours_poly[i], 3, false );
//      boundRect[i] = boundingRect( contours_poly[i] );
//      minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
//  }
//  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//  for( size_t i = 0; i< contours.size(); i++ )
//  {
//      Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//      drawContours( drawing, contours_poly, (int)i, color );
//      //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
//      //circle( drawing, centers[i], (int)radius[i], color, 2 );
//  }
//  imshow( "Contours", drawing );
//}

  //Line detector
//  CommandLineParser parser( argc, argv, "{@input | cluster.png | input image}" );
//  src = imread( samples::findFile( parser.get<String>( "@input" ) ), IMREAD_COLOR ); // Load an image
//  if( src.empty() )
//  {
//    std::cout << "Could not open or find the image!\n" << std::endl;
//    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
//    return -1;
//  }
//  dst.create( src.size(), src.type() );
//  cvtColor( src, src_gray, COLOR_BGR2GRAY );
//  namedWindow( window_name, WINDOW_AUTOSIZE );
//  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
//  CannyThreshold(0, 0);
//  waitKey(0);

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

//Costmap

//  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
//  {
//    printUsage (argv[0]);
//    return 0;
//  }
//  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
//  {
//    setUnseenToMaxRange = true;
//    std::cout << "Setting unseen values in range image to maximum range readings.\n";
//  }
//  if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
//    std::cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
//  int tmp_coordinate_frame;
//  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
//  {
//    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
//    std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
//  }
//  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
//    std::cout << "Setting support size to "<<support_size<<".\n";
//  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
//    std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
//  angular_resolution = pcl::deg2rad (angular_resolution);

//  // ------------------------------------------------------------------
//  // -----Read pcd file or create example point cloud if not given-----
//  // ------------------------------------------------------------------
//  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
//  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
//  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
//  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
//  if (!pcd_filename_indices.empty ())
//  {
//    std::string filename = argv[pcd_filename_indices[0]];
//    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
//    {
//      std::cerr << "Was not able to open file \""<<filename<<"\".\n";
//      printUsage (argv[0]);
//      return 0;
//    }
//    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
//                                                               point_cloud.sensor_origin_[1],
//                                                               point_cloud.sensor_origin_[2])) *
//                        Eigen::Affine3f (point_cloud.sensor_orientation_);
//    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
//    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
//      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
//  }
//  else
//  {
//    setUnseenToMaxRange = true;
//    std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
//    for (float x=-0.5f; x<=0.5f; x+=0.01f)
//    {
//      for (float y=-0.5f; y<=0.5f; y+=0.01f)
//      {
//        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
//        point_cloud.push_back (point);
//      }
//    }
//    point_cloud.width = point_cloud.size ();  point_cloud.height = 1;
//  }

//  // -----------------------------------------------
//  // -----Create RangeImage from the PointCloud-----
//  // -----------------------------------------------
//  float noise_level = 0.0;
//  float min_range = 0.0f;
//  int border_size = 1;
//  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
//  pcl::RangeImage& range_image = *range_image_ptr;
//  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
//  range_image.integrateFarRanges (far_ranges);
//  if (setUnseenToMaxRange)
//    range_image.setUnseenToMaxRange ();

//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//  viewer.setBackgroundColor (1, 1, 1);
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
//  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
//  //viewer.addCoordinateSystem (1.0f, "global");
//  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
//  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
//  viewer.initCameraParameters ();
//  setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

//  // --------------------------
//  // -----Show range image-----
//  // --------------------------
//  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//  range_image_widget.showRangeImage (range_image);

//  // --------------------------------
//  // -----Extract NARF keypoints-----
//  // --------------------------------
//  pcl::RangeImageBorderExtractor range_image_border_extractor;
//  pcl::NarfKeypoint narf_keypoint_detector;
//  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
//  narf_keypoint_detector.setRangeImage (&range_image);
//  narf_keypoint_detector.getParameters ().support_size = support_size;

//  pcl::PointCloud<int> keypoint_indices;
//  narf_keypoint_detector.compute (keypoint_indices);
//  std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

//  // ----------------------------------------------
//  // -----Show keypoints in range image widget-----
//  // ----------------------------------------------
// //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
//    //range_image_widget.markPoint (keypoint_indices[i]%range_image.width,
//                                  //keypoint_indices[i]/range_image.width);

//  // -------------------------------------
//  // -----Show keypoints in 3D viewer-----
//  // -------------------------------------
//  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
//  keypoints.resize (keypoint_indices.size ());
//  for (std::size_t i=0; i<keypoint_indices.size (); ++i)
//    keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
//  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

//  // ------------------------------------------------------
//  // -----Extract NARF descriptors for interest points-----
//  // ------------------------------------------------------
//  std::vector<int> keypoint_indices2;
//  keypoint_indices2.resize (keypoint_indices.size ());
//  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
//    keypoint_indices2[i]=keypoint_indices[i];
//  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
//  narf_descriptor.getParameters ().support_size = support_size;
//  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
//  pcl::PointCloud<pcl::Narf36> narf_descriptors;
//  narf_descriptor.compute (narf_descriptors);
//  std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
//                      <<keypoint_indices.size ()<< " keypoints.\n";

//  //--------------------
//  // -----Main loop-----
//  //--------------------
//  while (!viewer.wasStopped ())
//  {
//    range_image_widget.spinOnce ();  // process GUI events
//    viewer.spinOnce ();
//    pcl_sleep(0.01);
//  }
//}

  return 0;

}

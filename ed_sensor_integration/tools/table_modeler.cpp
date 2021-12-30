#include <pcl/point_types.h>
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
//#include <pcl/sample_consensus/ransac.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <boost/filesystem/convenience.hpp>

#include <ed/io/json_reader.h>
#include <ed/serialization/serialization.h>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <fstream>

//#include "ed_sensor_integration/sac_model_rectangle.h"
#include "ed_sensor_integration/sac_model_double_line.h"
#include "ed_sensor_integration/sac_model_circle.h"
#include "ed_sensor_integration/sac_model_horizontal_plane.h"

void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, Eigen::Matrix4f &final_transform)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

    *src = *cloud_src;
    *tgt = *cloud_tgt;


    //
    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (1);
    // Set the point representation
    // reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (src);
    reg.setInputTarget (tgt);



    //
    // Run the optimization
    Eigen::Matrix4f targetToSource;
    reg.setMaximumIterations (60);

    reg.align (*src);


        //
    // Get the transformation from target to source
    targetToSource = reg.getFinalTransformation();//.inverse();

    final_transform = targetToSource;
 }

float FilterPlane (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out) {
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
	
	
	
	/*
	Eigen::Vector3f ax(0, 0, 1);	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setAxis (ax);
	seg.setEpsAngle (0);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		*out = *cloud;
		return(-1);
	}
	// Extract the inliers
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*out);
    
	float Height = coefficients->values[3]; //final coefficient gives distance to the origin. normal vector is given as (0, 0, 1)
	return(Height);*/
}

void Segment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y, float z, std::vector<float> &tableHeight) {
    std::cout << "starting segmentation" << std::endl;
    std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; //using euclidian cluster extraction
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((*cloud).size()/100);
    ec.setMaxClusterSize ((*cloud).size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::cout << "obtained cluster indices" <<std::endl;
	
	//find closest cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out;
    float mindist = INFINITY;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != 
	cluster_indices.end (); ++it) //iterate through all clusters
    {
		//construct cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        float sumx = 0, sumy = 0, sumz = 0, dist = 0;
        for (uint j=0; j < (*cloud_cluster).width; ++j)
        {
			//sum up all points
            sumx += (*cloud_cluster)[j].x;
            sumy += (*cloud_cluster)[j].y;
            sumz += (*cloud_cluster)[j].z;
        }
        //find distance from camera to the middle of the cluster
        dist = pow((sumx/(*cloud_cluster).width-x),2) + pow((sumy/(*cloud_cluster).width-y),2) + pow((sumz/(*cloud_cluster).width-z),2);
        std::cout << "distance is " << sqrt(dist) << std::endl;
        if (dist < mindist) //check if closest so far
        {
            std::cout << "updating closest cluster" << std::endl;
            mindist = dist;
            cloud_out = cloud_cluster;
            //currentTableHeight = maxz
        }
    }
	
	float height = FilterPlane(cloud_out, cloud_out);
	if (height != -1)
	{
		tableHeight.push_back(height);
	}
	
    std::cout << "writing closest cluster" << std::endl;
    *cloud = *cloud_out;
}

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

    /*
    //Temporary fix for wrong rotational matrix
    Eigen::Matrix4f Correction = Eigen::Matrix4f::Identity();
    Correction(1,1) = -1;
    Correction(2,2) = -1;
    Transform = Transform * Correction.inverse();
    */

    std::cout << Transform << std::endl;
    return Transform;
}

pcl::PointCloud<pcl::PointXYZ> Flatten(pcl::PointCloud<pcl::PointXYZRGB> cloud) {
    float totx = 0, toty = 0, avgx, avgy;

    pcl::PointCloud<pcl::PointXYZ>::Ptr flat(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity ();
    std::cout << "Cloud width = " << (cloud).width << std::endl;
    (*flat).width = (cloud).width;
    (*flat).resize((*flat).width);
    for (uint i=0; i < (cloud).width; ++i)
    {
        //std::cout << "Writing point " << i << std::endl;
        (*flat)[i].x = cloud[i].x;
        (*flat)[i].y = cloud[i].y;
        (*flat)[i].z = 0;

        totx += (*flat)[i].x;
        toty += (*flat)[i].y;
    }
    Transform(0,3) = -totx/(*flat).width;
    Transform(1,3) = -toty/(*flat).width;

    pcl::transformPointCloud (*flat, *flat, Transform);

    pcl::ConcaveHull<pcl::PointXYZ> CHull;
    CHull.setInputCloud(flat);
    CHull.setAlpha (0.1);
    CHull.setDimension(2);
    CHull.reconstruct(*flat);
    return *flat;
}

Eigen::VectorXf Fit(pcl::PointCloud<pcl::PointXYZ> cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_ptr = cloud;
	float threshold = 0.03;
	// Create SAC model
	pcl::SampleConsensusModelDoubleLine<pcl::PointXYZ>::Ptr line1 (new pcl::SampleConsensusModelDoubleLine<pcl::PointXYZ>(cloud_ptr));

	// Create SAC method
	pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr sac1 (new pcl::RandomSampleConsensus<pcl::PointXYZ> (line1, threshold));
	sac1->setMaxIterations(10000);
	sac1->setProbability(1);

	// Fit model
	sac1->computeModel();

	// Get inliers
	std::vector<int> inliers1;
	sac1->getInliers(inliers1);

	// Get the model coefficients
	Eigen::VectorXf coeff1;
	sac1->getModelCoefficients (coeff1);
	
	
	//extract the inliers and fit second set of lines perpendicular
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr line1_inliers (new pcl::PointIndices);
	line1_inliers->indices = inliers1;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud_ptr);
	extract.setIndices (line1_inliers);
	extract.setNegative (true);
	extract.filter (*cloud2);
	// Create second SAC model
	pcl::SampleConsensusModelDoubleLine<pcl::PointXYZ>::Ptr line2 (new pcl::SampleConsensusModelDoubleLine<pcl::PointXYZ>(cloud2));
	// Create SAC method
	pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr sac2 (new pcl::RandomSampleConsensus<pcl::PointXYZ> (line2, threshold));
	sac2->setMaxIterations(10000);
	sac2->setProbability(1);
	// Fit model
	sac2->computeModel();
	// Get inliers
	std::vector<int> inliers2;
	sac2->getInliers(inliers2);
	// Get the model coefficients
	Eigen::VectorXf coeff2;
	sac2->getModelCoefficients (coeff2);

	pcl::io::savePCDFileASCII ("lines.pcd", *cloud2);
	
	pcl::PointIndices::Ptr line2_inliers (new pcl::PointIndices);
	line2_inliers->indices = inliers2;
	extract.setInputCloud (cloud2);
	extract.setIndices (line2_inliers);
	extract.filter (*cloud2);
	pcl::io::savePCDFileASCII ("outliers.pcd", *cloud2);
	
	
	//repeat above steps for a circle
	pcl::SampleConsensusModelCircle<pcl::PointXYZ>::Ptr circle (new pcl::SampleConsensusModelCircle<pcl::PointXYZ>(cloud_ptr));

	// Create SAC method
	pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr saccirc (new pcl::RandomSampleConsensus<pcl::PointXYZ> (circle, threshold));
	saccirc->setMaxIterations(10000);
	saccirc->setProbability(1);

	// Fit model
	saccirc->computeModel();

	// Get inliers
	std::vector<int> inliers3;
	saccirc->getInliers(inliers3);

	// Get the model coefficients
	Eigen::VectorXf coeff3;
	saccirc->getModelCoefficients (coeff3);
	
	float min_inliers = 0.0;
	Eigen::VectorXf output;
	if ((inliers1.size()+inliers2.size() >= inliers3.size()) && (static_cast<float>(inliers1.size()+inliers2.size())/static_cast<float>(cloud.size()) > min_inliers))
	{
		float w = coeff1[2];
		float l = coeff2[2];
		float r = coeff1[3] - coeff2[3];//difference in angle: if not 90 degrees, model is a parallellogram	
		
		output.resize(3);
		output[0] = l;
		output[1] = w;
		output[2] = r;
	}
	else if ((inliers1.size()+inliers2.size() < inliers3.size()) && (static_cast<float>(inliers3.size())/static_cast<float>(cloud.size()) > min_inliers))
	{
		output.resize(1);
		output[0] = coeff3[2];
	}
	return (output);
}

int main(int argc, char **argv) {

    // open pcd files

    if (argc < 2)
    {
        std::cout << "Usage:\n\n   table_modeler FILENAME1 FILENAME2 ...\n\n";
        return 1;
    }

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputs[argc-1]; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > inputs;

    std::vector<int> indices;

    std::cout << "starting to open" << std::endl;

    for (int i = 1; i < argc; ++i)
    {
        std::string name = std::string(argv[i]);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::cout << "attempting to open " << name << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name, *m) == -1) //* load the file
        {
            //PCL_ERROR ("Couldn't read file" + name + "\n");
            return -1;
        }
        std::cout << "opened " << name << std::endl;
        pcl::removeNaNFromPointCloud(*m, *m, indices);
        std::cout << "removed NAN from " << name << std::endl;
        inputs.push_back(m);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    std::vector<float> tableHeight;//used to keep track of the table height

    float x, y, z;//used to store camera position
    pcl::transformPointCloud (*inputs[0], *inputs[0], ReadJson(argv[1], &x, &y, &z));

    // Filter out floor
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.1)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (inputs[0]);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*inputs[0]);
    (*inputs[0]).is_dense = false;
    pcl::removeNaNFromPointCloud(*inputs[0], *inputs[0], indices);

    Segment(inputs[0], x, y, z, tableHeight);

    *result = *inputs[0];

    for (int i = 1; i < argc-1; ++i)
    {
        std::cout << "iteration " << i << std::endl;   

        // align to world model coordinates
        pcl::transformPointCloud (*inputs[i], *inputs[i], ReadJson(argv[i+1], &x, &y, &z));

        //filter out floor
        condrem.setInputCloud (inputs[i]);
        // apply filter
        condrem.filter (*inputs[i]);
        (*inputs[i]).is_dense = false;
        pcl::removeNaNFromPointCloud(*inputs[i], *inputs[i], indices);

        Segment(inputs[i], x, y, z, tableHeight);

        // align to previous file

        source = inputs[i];
        //target = inputs[i];

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

        std::cout << "aligning cloud " << i << " to cloud " << i - 1 << std::endl;
        pairAlign (source, result, pairTransform);
        std::cout << "transformation matrix =" << std::endl << pairTransform << std::endl;

        GlobalTransform *= pairTransform;

        pcl::transformPointCloud (*source, *temp, pairTransform);

        *result += *temp;
    }

    pcl::PointCloud<pcl::PointXYZ> flat;
    flat = Flatten(*result);
    std::cout << flat.width << std::endl;
    
    Eigen::VectorXf model = Fit(flat);
    
    std::cout << "model coefficients: " << std::endl << model << std::endl;

    std::cout << "Writing clouds" << std::endl;
    pcl::io::savePCDFileASCII ("combined.pcd", *result);
    pcl::io::savePCDFileASCII ("flat.pcd", flat);
    std::cout << "The table is " << std::accumulate(tableHeight.begin(), tableHeight.end(), 0.0) / tableHeight.size() << "m tall" << std::endl;
    
    

    return 0;
}

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <boost/filesystem/convenience.hpp>

#include <ed/io/json_reader.h>
#include <ed/serialization/serialization.h>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <fstream>

void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

    src = cloud_src;
    tgt = cloud_tgt;


    //
    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
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
    targetToSource = reg.getFinalTransformation().inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);



    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
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

    for (int i = 1; i < argc-1; ++i)
    {
        std::cout << "iteration " << i << std::endl;

        std::string pcd_filename = argv[i];
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
            return 0;
        }

        tue::config::Reader r(meta_data);
        // Read sensor pose
        geo::Pose3D sensor_pose;
        if (!ed::deserialize(r, "sensor_pose", sensor_pose))
        {
            std::cerr << "No field 'sensor_pose' specified." << std::endl;
            return 0;
        }

        std::cout << "x: " << sensor_pose.t.x << ", y: " << sensor_pose.t.y << std::endl;
        float x = sensor_pose.t.x;
        float y = sensor_pose.t.y;
        float z = sensor_pose.t.z;


        float qx, qy, qz, qw;

        const float n = 2.0f/(qx*qx+qy*qy+qz*qz+qw*qw);
        Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();/* {
            {1.0f - n*qy*qy - n*qz*qz, n*qx*qy - n*qz*qw, n*qx*qz + n*qy*qw, x},
            {n*qx*qy + n*qz*qw, 1.0f - n*qx*qx - n*qz*qz, n*qy*qz - n*qx*qw, y},
            {n*qx*qz - n*qy*qw, n*qy*qz + n*qx*qw, 1.0f - n*qx*qx - n*qy*qy, z},
            {0.0f, 0.0f, 0.0f, 1.0f}}; */

        Transform(0,0) = 1.0f - n*qy*qy - n*qz*qz;
        Transform(0,1) = n*qx*qy - n*qz*qw;
        Transform(0,2) = n*qx*qz + n*qy*qw;
        Transform(0,3) = x;
        Transform(1,0) = n*qx*qy + n*qz*qw;
        Transform(1,1) = 1.0f - n*qx*qx - n*qz*qz;
        Transform(1,2) = n*qy*qz - n*qx*qw;
        Transform(1,3) = y;
        Transform(2,0) = n*qx*qz - n*qy*qw;
        Transform(2,1) = n*qy*qz + n*qx*qw;
        Transform(2,2) = 1.0f - n*qx*qx - n*qy*qy;
        Transform(2,3) = z;

        Transform = Transform.inverse();

        pcl::transformPointCloud (*inputs[i], *inputs[i], Transform);

        //std::cout << metadata["rgbd_filename"] << std::endl;

        // align to world model coordinates

        // align to previous file

        source = inputs[i-1];
        target = inputs[i];

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

        std::cout << "aligning cloud " << i << " to cloud " << i - 1 << std::endl;
        pairAlign (source, target, temp, pairTransform);

        pcl::transformPointCloud (*temp, *result, GlobalTransform);

        GlobalTransform *= pairTransform;

        pcl::io::savePCDFileASCII ("combined.pcd", *result);
    }

    return 0;
}

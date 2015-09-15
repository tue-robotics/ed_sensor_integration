#include <ros/init.h>
#include <ros/node_handle.h>

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include <opencv2/highgui/highgui.hpp>

#include <fstream>

// ----------------------------------------------------------------------------------------------------

bool readImage(const std::string& filename, rgbd::Image& image, geo::Pose3D& sensor_pose)
{
    std::ifstream f_in;
    f_in.open(filename.c_str());

    if (!f_in.is_open())
    {
        std::cerr << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << f_in.rdbuf();
    std::string json = buffer.str();

    ed::io::JSONReader r(json.c_str());

    std::string rgbd_filename;
    if (!r.readValue("rgbd_filename", rgbd_filename))
    {
        std::cerr << "No field 'rgbd_filename' specified." << std::endl;
        return false;
    }

    if (r.readGroup("sensor_pose"))
    {
        ed::deserialize(r, sensor_pose);
        r.endGroup();
    }
    else
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

    std::ifstream f_rgbd;
    f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

    if (!f_rgbd.is_open())
    {
        std::cerr << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    tue::serialization::InputArchive a_in(f_rgbd);
    rgbd::deserialize(a_in, image);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY WORLD-MODEL-NAME" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_segmenter");
    ros::NodeHandle nh;

    if (argc != 3)
    {
        usage();
        return 1;
    }

    ed::UpdateRequest req;

    std::string model_name = argv[2];
    ed::models::ModelLoader model_loader;
    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error))
    {
        std::cerr << "Model '" << model_name << "' could not be loaded:" << std::endl << std::endl;
        std::cerr << error.str() << std::endl;
        return 1;
    }

    // Create world
    ed::WorldModel world_model;
    world_model.update(req);

    tue::filesystem::Path path = argv[1];
    tue::filesystem::Crawler crawler(path);

    Updater updater;

    tue::filesystem::Path filename;
    while(crawler.nextPath(filename))
    {
        if (filename.extension() != ".json")
            continue;

        rgbd::Image image;
        geo::Pose3D sensor_pose;

        if (!readImage(filename.string(), image, sensor_pose))
        {
            std::cerr << "Could not read " << filename << std::endl;
            continue;
        }

        ed::UpdateRequest update_req;
        UpdateResult res(update_req);
        updater.update(world_model, image, sensor_pose, "on_top_of cabinet", res);

        std::string error_msg = res.error.str();
        if (!error_msg.empty())
        {
            std::cout << error_msg << std::endl;
            continue;
        }

        std::cout << sensor_pose << std::endl;

        cv::Mat rgb = image.getRGBImage().clone();

        for(unsigned int i = 0; i < res.entity_updates.size(); ++i)
        {
            const EntityUpdate& e_update = res.entity_updates[i];
            std::cout << e_update.id << std::endl;

            for(std::vector<unsigned int>::const_iterator it = e_update.pixel_indices.begin(); it != e_update.pixel_indices.end(); ++it)
            {
                rgb.at<cv::Vec3b>(*it) = cv::Vec3b(0, 0, 255);
            }
        }
        std::cout << std::endl;

        cv::imshow("RGB", rgb);
        cv::waitKey();
    }

   return 0;
}

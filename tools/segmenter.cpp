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

struct Snapshot
{
    rgbd::Image image;
    geo::Pose3D sensor_pose;
};

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

bool loadWorldModel(const std::string& model_name, ed::WorldModel& world_model)
{
    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;

    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error))
    {
        std::cerr << "Model '" << model_name << "' could not be loaded:" << std::endl << std::endl;
        std::cerr << error.str() << std::endl;
        return false;
    }

    // Reset world
    world_model = ed::WorldModel();

    // Update world
    world_model.update(req);

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

    std::string model_name = argv[2];

    ed::WorldModel world_model;
    if (!loadWorldModel(model_name, world_model))
        return false;

    tue::filesystem::Path path = argv[1];
    tue::filesystem::Crawler crawler;

    if (path.isDirectory())
        crawler.setRootPath(path);
    else
        crawler.setRootPath(path.parentPath());

    Updater updater;

    std::vector<Snapshot> snapshots;
    unsigned int i_snapshot = 0;

    while(true)
    {
        if (i_snapshot >= snapshots.size())
        {
            bool file_found = true;
            tue::filesystem::Path filename;

            if (path.isRegularFile() && snapshots.empty())
                filename = path;
            else
            {
                file_found = false;
                while (crawler.nextPath(filename))
                {
                    if (filename.extension() == ".json")
                    {
                        file_found = true;
                        break;
                    }
                }
            }

            if (file_found)
            {
                i_snapshot = snapshots.size();
                snapshots.push_back(Snapshot());
                Snapshot& snapshot = snapshots.back();

                if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose))
                {
                    std::cerr << "Could not read " << filename << std::endl;
                    continue;
                }
            }
            else
            {
                if (snapshots.empty())
                    break;

                i_snapshot = snapshots.size() - 1;
            }
        }

        Snapshot& snapshot = snapshots[i_snapshot];

        ed::UpdateRequest update_req;
        UpdateResult res(update_req);
        updater.update(world_model, snapshot.image, snapshot.sensor_pose, "on_top_of cabinet", res);

        std::string error_msg = res.error.str();
        if (!error_msg.empty())
        {
            std::cerr << error_msg << std::endl;
            continue;
        }

        cv::Mat rgb = snapshot.image.getRGBImage().clone();

        for(unsigned int i = 0; i < res.entity_updates.size(); ++i)
        {
            const EntityUpdate& e_update = res.entity_updates[i];
            for(std::vector<unsigned int>::const_iterator it = e_update.pixel_indices.begin(); it != e_update.pixel_indices.end(); ++it)
            {
                rgb.at<cv::Vec3b>(*it) = cv::Vec3b(0, 0, 255);
            }
        }

        cv::imshow("RGB", rgb);
        char key = cv::waitKey();

        if (key == 81)  // Left arrow
        {
            if (i_snapshot > 0)
                --i_snapshot;
        }
        else if (key == 83) // Right arrow
        {
            ++i_snapshot;
        }
        else if (key == 'r')
        {
            // Reload
            loadWorldModel(model_name, world_model);
        } else if (key == 'q')
        {
            break;
        }

//        std::cout << (int)key << std::endl;
    }

    return 0;
}

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
    std::string area_description;
};

// ----------------------------------------------------------------------------------------------------

bool readImage(const std::string& filename, rgbd::Image& image, geo::Pose3D& sensor_pose, std::string& area_description)
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

    if (!r.readValue("area", area_description))
        area_description.clear();

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
        return 1;

    tue::filesystem::Path path = argv[1];
    if (!path.exists())
    {
        std::cerr << "Path '" << path << "' does not exist." << std::endl;
        return 1;
    }

    tue::filesystem::Crawler crawler;

    if (path.isDirectory())
        crawler.setRootPath(path);
    else
        crawler.setRootPath(path.parentPath());

    Updater updater;

    std::vector<Snapshot> snapshots;
    unsigned int i_snapshot = 0;

    while(ros::ok())
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

                if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose, snapshot.area_description))
                {
                    std::cerr << "Could not read " << filename << std::endl;
                    snapshots.pop_back();
                    continue;
                }
                else
                {
//                    std::cout << "Successfully loaded " << filename << std::endl;
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
        updater.update(world_model, snapshot.image, snapshot.sensor_pose, snapshot.area_description, res);

        cv::Mat canvas = snapshot.image.getRGBImage().clone();

        std::string error_msg = res.error.str();
        if (error_msg.empty())
        {
            int depth_width = snapshot.image.getDepthImage().cols;
            double f = (double)canvas.cols / depth_width;

            for(unsigned int i = 0; i < res.entity_updates.size(); ++i)
            {
                const EntityUpdate& e_update = res.entity_updates[i];
                if (e_update.pixel_indices.empty())
                    continue;

                unsigned i_pxl = e_update.pixel_indices[0];
                cv::Point bb_min(i_pxl % depth_width, i_pxl / depth_width);
                cv::Point bb_max(i_pxl % depth_width, i_pxl / depth_width);

                for(std::vector<unsigned int>::const_iterator it = e_update.pixel_indices.begin(); it != e_update.pixel_indices.end(); ++it)
                {
                    int x = *it % depth_width;
                    int y = *it / depth_width;

                    bb_min.x = std::min(bb_min.x, x);
                    bb_min.y = std::min(bb_min.y, y);
                    bb_max.x = std::max(bb_max.x, x);
                    bb_max.y = std::max(bb_max.y, y);

                    for(double x2 = f * x; x2 < (f * (x + 1)); ++x2)
                        for(double y2 = f * y; y2 < (f * (y + 1)); ++y2)
                            canvas.at<cv::Vec3b>(y2, x2) = cv::Vec3b(0, 0, 255);
                }

                cv::Point d(2, 2);
                cv::rectangle(canvas, f * bb_min, f * bb_max, cv::Scalar(255, 255, 255), 2);
                cv::rectangle(canvas, f * bb_min - d, f * bb_max + d, cv::Scalar(0, 0, 0), 2);
                cv::rectangle(canvas, f * bb_min + d, f * bb_max - d, cv::Scalar(0, 0, 0), 2);
            }

        }
        else
        {
            std::cerr << error_msg << std::endl;
            cv::putText(canvas, "Segmentation failed", cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);
        }

        cv::imshow("RGB", canvas);
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

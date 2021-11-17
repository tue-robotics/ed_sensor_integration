#include <ros/init.h>
#include <ros/node_handle.h>

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/image.h>
#include <rgbd/serialization.h>
#include <rgbd/view.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <ed/entity.h>

#include <fstream>
#include <vector>

// ----------------------------------------------------------------------------------------------------

struct Snapshot
{
    rgbd::ImagePtr image;
    geo::Pose3D sensor_pose;
    std::string area_description;
    ed::WorldModel world_model;
};

/**
 * read an rgbd image from file
 *
 * @param[in] filename name of the json file that describes the image
 * @param[out] image image to write to
 * @param[out] sensor_pose pose to write to
 * @return bool whether image was successfully loaded
 */
bool readImage(const std::string& filename, rgbd::ImagePtr& image, geo::Pose3D& sensor_pose)
{
    tue::config::DataPointer meta_data;

    try
    {
        meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;
        return false;
    }

    tue::config::Reader r(meta_data);

    // Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

        std::ifstream f_rgbd;
        f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

        if (!f_rgbd.is_open())
        {
            std::cerr << "Could not open '" << filename << "'." << std::endl;
            return false;
        }

        image.reset(new rgbd::Image);

        tue::serialization::InputArchive a_in(f_rgbd);
        rgbd::deserialize(a_in, *image);
    }

    // Read sensor pose
    if (!ed::deserialize(r, "sensor_pose", sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    return true;
}

/**
 * Load a worldmodel from file using its name.
 *
 * @param[in] model_name name of the worldmodel
 * @param[out] world_model worldmodel to write to
 * @return bool whether model is successfully loaded
 */
bool loadWorldModel(const std::string& model_name, ed::WorldModel& world_model)
{
    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;

    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error, true))
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

/**
 * output the usage of the segmenter tool
 */
void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY [WORLDMODEL-NAME] [ENTITY-ID]" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_segmenter");
    ros::NodeHandle nh;

    if (argc != 4)
    {
        if (argc == 2)
        {
            std::cout << "No worldmodel provided! Using empty worldmodel and no segmentation area" << std::endl;
        }
        else
        {
            usage();
            return 1;
        }
    }

    tue::filesystem::Path path = argv[1];
    if (!path.exists())
    {
        std::cerr << "Path '" << path << "' does not exist." << std::endl;
        return 1;
    }

    ed::WorldModel world_model;
    std::string entity_id;
    ed::EntityConstPtr e;

    if (argc == 4)
    {
        std::string model_name = argv[2];

        if (!loadWorldModel(model_name, world_model))
        {
            std::cerr << "World model '" << model_name << "' could not be loaded." << std::endl;
            return 1;
        }

        entity_id = argv[3];
        e = world_model.getEntity(entity_id);
        if (!e)
        {
            std::cerr << "Entity '" << entity_id << "' could not be found in world model '" << model_name << "'." << std::endl;
            return 1;
        }
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
                snapshot.world_model = world_model;
                if (e)
                    snapshot.area_description = std::string("on_top_of ").append(entity_id);
                else
                    snapshot.area_description = "";

                std::cout << "area description" << snapshot.area_description << std::endl;

                if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose))
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

        UpdateRequest kinect_update_request;
        kinect_update_request.area_description = snapshot.area_description;
        updater.update(snapshot.world_model, snapshot.image, snapshot.sensor_pose, kinect_update_request, res);

        std::cout << update_req.measurements.size() << std::endl;

        cv::Mat canvas = snapshot.image->getRGBImage().clone();

        std::string error_msg = res.error.str();
        if (error_msg.empty())
        {
            int depth_width = snapshot.image->getDepthImage().cols;
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
        else if (key == 'q')
        {
            break;
        }

//        std::cout << (int)key << std::endl;
    }

    return 0;
}

#include <ros/init.h>
#include <ros/node_handle.h>

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>
#include <ed/kinect/fitter.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/image.h>
#include <rgbd/serialization.h>
#include <rgbd/view.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <ed/entity.h>

#include "geolib/math_types.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

// ----------------------------------------------------------------------------------------------------

bool readImage(const std::string& filename, rgbd::ImagePtr& image, geo::Pose3D& sensor_pose,
               ed::WorldModel& world_model, std::string& area_description)
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

//    if (r.hasError())
//    {
//        std::cout << "Error while reading file '" << filename << "':\n\n" << r.error() << std::endl;
//        return false;
//    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY  WORLDMODEL_NAME  ENTITY_ID" << std::endl;
}

// Getting roll, pitch and yaw from a quaternion,
// copied from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

void drawLine(
        cv::Mat& canvas, geo::Vec2 point1, geo::Vec2 point2, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar color)
{
    // computing points relative to the pose (in meters)
    geo::Vec2 rel_point1 = pose * point1;
    geo::Vec2 rel_point2 = pose * point2;

    // position to pixels
    int x_p1 = origin_x + (int)(rel_point1.x * resolution);
    int y_p1 = origin_y - (int)(rel_point1.y * resolution);
    int x_p2 = origin_x + (int)(rel_point2.x * resolution);
    int y_p2 = origin_y - (int)(rel_point2.y * resolution);

    if (x_p1 < 0 || x_p2 < 0|| x_p1 >= canvas.cols || x_p2 >= canvas.cols)
    {
        std::cout << "Entity: x-coordinate out of range" << std::endl;
        return;
    }
    if (y_p1 < 0 || y_p2 < 0|| y_p1 >= canvas.rows || y_p2 >= canvas.rows)
    {
        std::cout << "Entity: y-coordinate out of range" << std::endl;
        return;
    }

    // paint to screen
    cv::Point point1_p(x_p1, y_p1);
    cv::Point point2_p(x_p2, y_p2);
    cv::line(canvas, point1_p, point2_p, color, 1);
}

void drawShape2D(cv::Mat& canvas, const Shape2D& shape, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar color)
{
    for (unsigned int i=0; i < shape.size(); i++)
    {
        for (unsigned int j=0; j < shape[i].size()-1; j++)
        {
            drawLine(canvas, shape[i][j], shape[i][j+1], pose, resolution, origin_x, origin_y, color);
        }
        drawLine(canvas, shape[i][0], shape[i][shape[0].size()-1], pose, resolution, origin_x, origin_y, color);
    }

    // paint entity center

    int x_p_ent = origin_x + (int)(pose.t.x * resolution);
    int y_p_ent = origin_y - (int)(pose.t.y * resolution);

    cv::Point Entity_center(x_p_ent, y_p_ent);
    cv::circle(canvas, Entity_center, 3, color, cv::FILLED);

}
// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_fitter");
    ros::NodeHandle nh;

    if (argc != 4)
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

    std::string entity_id = argv[3];


    tue::filesystem::Crawler crawler;

    if (path.isDirectory())
        crawler.setRootPath(path);
    else
        crawler.setRootPath(path.parentPath());

    Updater updater;
    Fitter fitter;

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

                if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose,
                               snapshot.world_model, snapshot.area_description))
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

        FitterData fitterdata;
        geo::Pose3D fitted_pose;
        EstimationInputData result;

        ed::EntityConstPtr e = snapshot.world_model.getEntity(entity_id);

        fitter.configureBeamModel(snapshot.image->getCameraModel());
        fitter.processSensorData(*snapshot.image, snapshot.sensor_pose, fitterdata);

        bool estimateEntityPose = fitter.estimateEntityPose(fitterdata, snapshot.world_model, entity_id, e->pose(), fitted_pose);

        // show snapshot
        cv::Mat rgbcanvas = snapshot.image->getRGBImage().clone();
        cv::imshow("RGB", rgbcanvas);

        // visualise fitting (positive y direction = downwards)
        int canvas_width = 600;
        int canvas_height = 600;
        cv::Mat canvas = cv::Mat(canvas_height, canvas_width, CV_8UC3, cv::Scalar(0,0,0));

        // needed parameters: Fitterdata fitterdata;
        int sensor_x = canvas_width/2;
        int sensor_y = canvas_height * 8/10;
        float canvas_resolution = 100; //pixels per meter

        // paint to screen the location of HERO
        cv::Point sensorlocation(sensor_x, sensor_y);
        cv::Scalar sensorcolor(0, 255, 0); // green
        cv::circle(canvas, sensorlocation, 3, sensorcolor, cv::FILLED);

        geo::Transform2 sensor_pose2d = fitterdata.sensor_pose_xya.projectTo2d();
        geo::Transform2 entity_pose2d = e->pose().projectTo2d();
        geo::Transform2 fitted_pose2d = fitted_pose.projectTo2d();

        // paint entity (from worldmodel)
        EntityRepresentation2D entity_2d = fitter.GetOrCreateEntity2D(e);
        //geo::Pose3D relpose = fitterdata.sensor_pose_xya.inverse() * e->pose();
        geo::Transform2 relpose = sensor_pose2d.inverse() * entity_pose2d;
        cv::Scalar entitycolour(0, 255, 255);
        drawShape2D(canvas, entity_2d.shape_2d, relpose, canvas_resolution, sensor_x, sensor_y, entitycolour);

        if (estimateEntityPose)
        {
            // paint fitted entity
            cv::Scalar fittedcolour(243, 192, 15); // blue
            geo::Transform2 fitted_relpose = sensor_pose2d.inverse() * fitted_pose2d;
            drawShape2D(canvas, entity_2d.shape_2d, fitted_relpose, canvas_resolution, sensor_x, sensor_y, fittedcolour);
        }
        else
        {
            std::cout << "Fitted entity not printed in image because of a fitter error" << std::endl;
        }

        // paint sensor_ranges
        for (unsigned int i = 0; i < fitterdata.sensor_ranges.size(); ++i) {
            float fx_ = 2 * fitterdata.sensor_ranges.size() / 4; // Constant value from fitter.cpp
            float x_m = fitterdata.sensor_ranges[i] * ((static_cast<int>(i) - (fitterdata.sensor_ranges.size() / 2)) / fx_);
            float y_m = fitterdata.sensor_ranges[i];

            // postion to pixels
            int x_p = sensor_x + (int)(x_m * canvas_resolution);
            int y_p = sensor_y - (int)(y_m * canvas_resolution);

            if (x_p < 0 || x_p >= canvas_width)
                continue;
            if (y_p < 0 || y_p >= canvas_height)
                continue;
            if (fitterdata.sensor_ranges[i] == 0) // filter out sensor_ranges equal to zero
                continue;

            // paint to screen
            cv::Point centerCircle(x_p, y_p);
            cv::Scalar colourCircle(161, 17, 187);
            cv::circle(canvas, centerCircle, 2, colourCircle, cv::FILLED);
        }

        cv::imshow("Fitting", canvas);
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
    }

    return 0;
}

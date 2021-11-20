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

#include "ed_sensor_integration/tools/snapshot.h"


// visualization parameters
int canvas_width = 600; //pixels
int canvas_height = 600; // pixels
float canvas_resolution = 100; //pixels per meter

int sensor_x = canvas_width/2;
int sensor_y = canvas_height * 8/10;
cv::Point sensorlocation(sensor_x, sensor_y);

cv::Scalar sensor_colour(0, 255, 0); // green
cv::Scalar entity_colour(0, 255, 255); // yellow
cv::Scalar fitted_colour(243, 192, 15); // blue
cv::Scalar measurement_colour(161, 17, 187); // purple

void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY  WORLDMODEL_NAME  ENTITY_ID" << std::endl;
}

void drawLine(const cv::Mat& canvas, geo::Vec2 point1, geo::Vec2 point2, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar color)
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

void drawShape2D(const cv::Mat& canvas, const Shape2D& shape, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar color)
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

cv::Mat visualizeFitting(EntityRepresentation2D entity, geo::Transform2 sensor_pose, geo::Transform2 entity_pose,
                         geo::Transform2 fitted_pose, FitterData fitterdata, bool estimateEntityPose)
{
    // visualize fitting
    cv::Mat canvas = cv::Mat(canvas_height, canvas_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // paint to screen the location of the sensor
    cv::circle(canvas, sensorlocation, 3, sensor_colour, cv::FILLED);

    // paint entity (from worldmodel)
    geo::Transform2 relpose = sensor_pose.inverse() * entity_pose;
    drawShape2D(canvas, entity.shape_2d, relpose, canvas_resolution, sensor_x, sensor_y, entity_colour);

    if (estimateEntityPose)
    {
        // paint fitted entity
        geo::Transform2 fitted_relpose = sensor_pose.inverse() * fitted_pose;
        drawShape2D(canvas, entity.shape_2d, fitted_relpose, canvas_resolution, sensor_x, sensor_y, fitted_colour);
    }
    else
    {
        std::cout << "Fitted entity not printed in image because of a fitter error" << std::endl;
    }

    // paint sensor_ranges
    uint nranges = fitterdata.sensor_ranges.size();
    uint half_nranges = nranges/2;
    float fx = fitterdata.fx;
    for (unsigned int i = 0; i < nranges; ++i)
    {
        float x_m = fitterdata.sensor_ranges[i] * ((static_cast< float >(i) - half_nranges) / fx);
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
        cv::circle(canvas, centerCircle, 2, measurement_colour, cv::FILLED);
    }

    return canvas;
}

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
    if (!loadWorldModel(model_name, world_model)){
        std::cerr << "World model '" << model_name << "' could not be loaded." << std::endl;
        return 1;
    }

    std::string entity_id = argv[3];
    ed::EntityConstPtr e = world_model.getEntity(entity_id);
    if (!e)
    {
        std::cerr << "Entity '" << entity_id << "' could not be found in world model '" << model_name << "'." << std::endl;
        return 1;
    }

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

        fitter.configureBeamModel(snapshot.image->getCameraModel());
        fitter.processSensorData(*snapshot.image, snapshot.sensor_pose, fitterdata);

        bool estimateEntityPose = fitter.estimateEntityPose(fitterdata, snapshot.world_model, entity_id, e->pose(), fitted_pose);

        // poses for visualization
        geo::Transform2 sensor_pose2d = fitterdata.sensor_pose_xya.projectTo2d();
        geo::Transform2 entity_pose2d = e->pose().projectTo2d();
        geo::Transform2 fitted_pose2d = fitted_pose.projectTo2d();

        // show snapshot
        cv::Mat rgbcanvas = snapshot.image->getRGBImage().clone();
        cv::imshow("RGB", rgbcanvas);

        EntityRepresentation2D entity_2d = fitter.GetOrCreateEntity2D(e);
        cv::Mat canvas = visualizeFitting(entity_2d, sensor_pose2d, entity_pose2d, fitted_pose2d, fitterdata, estimateEntityPose);
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

#include <ed/entity.h>
#include <ed/world_model.h>

#include <ed/kinect/fitter.h>
#include <ed/kinect/image_buffer.h>

#include <geolib/math_types.h>

#include <opencv2/highgui/highgui.hpp>

#include "ed_sensor_integration/tools/fitter_viz.h"
#include "ed_sensor_integration/tools/snapshot.h"


/**
 * @brief usage, print how the executable should be used and explain the input
 */
void usage()
{
    std::cout << "Usage: ed_fitter WORLDMODEL_NAME  ENTITY_ID" << std::endl;
}

/**
 * @brief main executable to visualise the fitting process of live images.
 * @param argc: should be 4
 * @return
 */
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }

    std::string model_name = argv[1];

    ed::WorldModelPtr world_model;
    try
    {
        world_model = ed::loadWorldModel(model_name);
    }
    catch (ed::ModelNotFoundException e)
    {
        std::cerr << "World model '" << model_name << "' could not be loaded." << std::endl;
        std::cerr << e.what() << std::endl;
        return 1;
    }

    std::string entity_id = argv[2];
    ed::EntityConstPtr e = world_model->getEntity(entity_id);
    if (!e)
    {
        std::cerr << "Entity '" << entity_id << "' could not be found in world model '" << model_name << "'." << std::endl;
        return 1;
    }

    ros::init(argc, argv, "fitting_visualizer");

    ImageBuffer image_buffer;
    image_buffer.initialize("/hero/head_rgbd_sensor/rgbd", "map");

    Fitter fitter;

    while(true)
    {
        rgbd::ImageConstPtr image;
        geo::Pose3D sensor_pose;

        if (!image_buffer.waitForRecentImage(image, sensor_pose, 2.0))
        {
            std::cerr << "No image received, will try again." << std::endl;
            continue;
        }

        FitterData fitterdata;
        geo::Pose3D fitted_pose;

        fitter.configureBeamModel(image->getCameraModel());
        fitter.processSensorData(*image, sensor_pose, fitterdata);

        bool estimateEntityPose = fitter.estimateEntityPose(fitterdata, *world_model, entity_id, e->pose(), fitted_pose);

        // poses for visualization
        geo::Transform2 sensor_pose2d = fitterdata.sensor_pose_xya.projectTo2d();
        geo::Transform2 entity_pose2d = e->pose().projectTo2d();
        geo::Transform2 fitted_pose2d = fitted_pose.projectTo2d();

        // show snapshot
        cv::Mat rgbcanvas = image->getRGBImage().clone();
        cv::imshow("RGB", rgbcanvas);

        EntityRepresentation2D entity_2d = fitter.GetOrCreateEntity2D(e);
        cv::Mat canvas = visualizeFitting(entity_2d, sensor_pose2d, entity_pose2d, fitted_pose2d, fitterdata, estimateEntityPose);
        cv::imshow("Fitting", canvas);
    }

    return 0;
}

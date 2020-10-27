#include <gtest/gtest.h>
#include <iostream>

// ROS
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

// TU/e Robotics
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/models/model_loader.h>
#include <ed/update_request.h>
#include <ed/uuid.h>
#include <ed/relations/transform_cache.h>
#include <geolib/Shape.h>
#include <geolib/sensors/DepthCamera.h>
#include "tue/config/reader_writer.h"
#include "tue/config/loaders/sdf.h"

// ED sensor integration
#include <ed/kinect/fitter.h>


/**
 * @brief setupRasterizer sets up the rasterizer
 * N.B.: shouldn't we move this somewhere else? It's being used more often
 * @param rasterizer
 */
void setupRasterizer(geo::DepthCamera& rasterizer)
{
    // Set cam model
    image_geometry::PinholeCameraModel cam_model;
    sensor_msgs::CameraInfo cam_info;
    cam_info.K = sensor_msgs::CameraInfo::_K_type({554.2559327880068, 0.0, 320.5,
                  0.0, 554.2559327880068, 240.5,
                  0.0, 0.0, 1.0});
    cam_info.P = sensor_msgs::CameraInfo::_P_type({554.2559327880068, 0.0, 320.5, 0.0,
                               0.0, 554.2559327880068, 240.5, 0.0,
                               0.0, 0.0, 1.0, 0.0});
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = 640;
    cam_info.height = 480;
    cam_model.fromCameraInfo(cam_info);

    rasterizer.setFocalLengths(cam_model.fx(), cam_model.fy());
    rasterizer.setOpticalCenter(cam_model.cx(), cam_model.cy());
    rasterizer.setOpticalTranslation(cam_model.Tx(), cam_model.Ty());

}


/**
 * @brief renderImage renders an rgbd image from a world model.
 * N.B.: copied from https://github.com/tue-robotics/fast_simulator/blob/master/src/Kinect.cpp
 * @param wm
 */
// ToDo: generalize
void renderImage(const geo::DepthCamera& rasterizer, const ed::WorldModel& wm, cv::Mat& depth_image)
{
    depth_image.setTo(0.0f);

    // Iterate over all entities
    std::cout << "Iterate over all entities" << std::endl;

    for(ed::WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
    {
        std::cout << "\nNext entity... " << std::endl;
        const ed::EntityConstPtr& e = *it;
        std::cout << "Entity ID: " << e->id() << std::endl;
        
//        std::cout << "Getting shaperevision" << std::endl;
//        int shaperevision = e->shapeRevision();
//        std::cout << "ShapeRevision: " << shaperevision << std::endl;
//        if (shaperevision == 0)
//        {
//            std::cout << "Entity " << e->id() << " does not have a shape, skipping..." << std::endl;
//            continue;
//        }

        std::cout << "Getting shape" << std::endl;
        geo::ShapeConstPtr shape_ptr = e->shape();
        if (shape_ptr)
            std::cout << "Entity " << e->id() << " has a shape of type " << shape_ptr->TYPE << std::endl;
        else
            std::cout << "Entity " << e->id() << " does not have a shape" << std::endl;

        std::cout << "Entity done" << std::endl;

        // Render ...
//        geo::Transform t(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//        rasterizer.rasterize(*shape, t, depth_image);

    }
    std::cout << "Iteration done" << std::endl;


//    const std::map<std::string, Object*>& objects = world.getObjects();
//    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
//        Object* obj = it_obj->second;

//        geo::ShapePtr shape = obj->getShape();
//        if (shape) {
//            geo::Transform t = tf_map_to_kinect.inverse() * obj->getAbsolutePose();
//            rasterizer_.rasterize(*shape, t, depth_image);
//        }

//        std::vector<Object*> children;
//        obj->getChildrenRecursive(children);

//        for(std::vector<Object*>::const_iterator it = children.begin(); it != children.end(); ++it) {
//            const Object& child = **it;
//            geo::ShapePtr child_shape = child.getShape();
//            if (child_shape) {
//                geo::Transform t = tf_map_to_kinect.inverse() * child.getAbsolutePose();
//                rasterizer_.rasterize(*child_shape, t, depth_image);
//            }
//        }
//    }

}


// ToDo: this has a lot of overlap with code in ed/tools/view_model.cpp. Generalize this
void createWorldModel(ed::WorldModel& wm)
{
    ed::models::ModelLoader loader;
    ed::UpdateRequest request;
//    request.setType("map", "waypoint");
//    request.setType("table", "object");

//    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
//    t1->insert(0, geo::Pose3D(0, 0, 0, 0, 0, 0));
//    request.setRelation("map", "table", t1);

    // Load the table model from the sdf file
    std::string path = ros::package::getPath("ed_sensor_integration");
    path += "/test/table.sdf";
    tue::config::ReaderWriter config;
    tue::config::loadFromSDFFile(path, config);
    std::cout << "Config:\n" << config << std::endl;

    std::stringstream errors;
    bool load_result = loader.create(config.data(), request, errors);
    ASSERT_TRUE(load_result);

    wm.update(request);

}


void moveFurnitureObject(const ed::UUID& id, ed::WorldModel& wm, float x, float y, float z, float roll, float pitch, float yaw)
{
    ed::UpdateRequest request;
    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
    t1->insert(0, geo::Pose3D(x, y, z, roll, pitch, yaw));
    request.setRelation("map", "table", t1);
    wm.update(request);

}


/**
 * @brief fitSupportingEntity fits a supporting entity
 * @param image depth image
 * @param sensor_pose pose of the camera when the image was taken
 * @param wm current world model
 * @param entity_id id of the entity that will be fitted
 * @param max_yaw_change max angle change
 * @param fitter
 * @param new_pose
 * @return success
 */
bool fitSupportingEntity(const rgbd::Image* image, const geo::Pose3D& sensor_pose,
                         const ed::WorldModel& wm, const ed::UUID& entity_id, const double max_yaw_change,
                         Fitter& fitter, geo::Pose3D& new_pose)
{
    // ToDo: create a function for this in the library
    // ToDo: does it make sense to provide RGBD data here or rather a more standard data type?
    FitterData fitterdata;
    fitter.processSensorData(*image, sensor_pose, fitterdata);

    ed::EntityConstPtr e = wm.getEntity(entity_id);
    // ToDo: what do we want to do if we cannot find the entity?
    if (!e)
        throw std::runtime_error("Entity not found in WM");
    return fitter.estimateEntityPose(fitterdata, wm, entity_id, e->pose(), new_pose, max_yaw_change);
}


TEST(TestSuite, testCase)
{
    // Setup world model
    std::cout << "Starting testsuite" << std::endl;
    ed::WorldModel wm;
    createWorldModel(wm);

    // Create rasterizer that is used for rendering of depth images
    geo::DepthCamera rasterizer;
    setupRasterizer(rasterizer);

    // Render image
    cv::Mat depth_image;
    renderImage(rasterizer, wm, depth_image);

    // Move the table
//    moveFurnitureObject("table", wm, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

//    // Render another image
//    cv::Mat depth_image2;
//    renderImage(rasterizer, wm, depth_image2);

//    // Start fitting

    std::cout << "Tests done" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

//  g_argc = argc;
//  g_argv = argv;

  return RUN_ALL_TESTS();
}

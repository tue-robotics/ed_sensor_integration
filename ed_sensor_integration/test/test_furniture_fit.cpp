#include <gtest/gtest.h>
#include <math.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

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
#include <ed/rendering.h>
#include <ed/uuid.h>
#include <ed/relations/transform_cache.h>
#include <geolib/Shape.h>
#include <geolib/sensors/DepthCamera.h>
#include <rgbd/image.h>
#include "tue/config/reader_writer.h"
#include "tue/config/loaders/sdf.h"

// ED sensor integration
#include <ed/kinect/fitter.h>

uint CAM_RESOLUTION_WIDTH = 640;
uint CAM_RESOLUTION_HEIGHT = 480;

bool SHOW_DEBUG_IMAGES = false;

// Getting roll, pitch and yaw from a quaternion,
// copied from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// N.B.: we should have a proper implemenation
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


double getYaw(const geo::Mat3& rotation)
{
    // Get quaternion
    geo::Quaternion geo_quaternion;
    rotation.getRotation(geo_quaternion);

    // Convert it to struct
    Quaternion quaternion;
    quaternion.x = geo_quaternion.getX();
    quaternion.y = geo_quaternion.getY();
    quaternion.z = geo_quaternion.getZ();
    quaternion.w = geo_quaternion.getW();

    // Get the Euler angles
    EulerAngles angles = ToEulerAngles(quaternion);
    std::cout << "Matrix: " << rotation << " --> yaw: " << angles.yaw << std::endl;
    return angles.yaw;
}


/**
 * @brief setupRasterizer sets up the rasterizer
 * N.B.: shouldn't we move this somewhere else? It's being used more often
 * @param rasterizer
 */
void setupRasterizer(image_geometry::PinholeCameraModel& cam_model, geo::DepthCamera& rasterizer)
{
    // Set cam model
    sensor_msgs::CameraInfo cam_info;
    cam_info.K = sensor_msgs::CameraInfo::_K_type({554.2559327880068, 0.0, 320.5,
                  0.0, 554.2559327880068, 240.5,
                  0.0, 0.0, 1.0});
    cam_info.P = sensor_msgs::CameraInfo::_P_type({554.2559327880068, 0.0, 320.5, 0.0,
                               0.0, 554.2559327880068, 240.5, 0.0,
                               0.0, 0.0, 1.0, 0.0});
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = CAM_RESOLUTION_WIDTH;
    cam_info.height = CAM_RESOLUTION_HEIGHT;
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
void renderImage(const geo::DepthCamera& rasterizer, const geo::Pose3D& cam_pose, const ed::WorldModel& wm, cv::Mat& depth_image)
{
    cv::Mat image(depth_image.rows, depth_image.cols, CV_8UC3, cv::Scalar(20, 20, 20));
    /// TESTCODE: DON'T COMMIT
    std::cout << "renderImage: getting shape and entity" << std::endl;
    std::cout << "WM revision: " << wm.revision() << ", entity revision: " << wm.getEntity("table")->revision() << std::endl;
    wm.getEntity(ed::UUID("table"))->shape();
    std::cout << "renderImage: getting shape and entity, alternative attempt" << std::endl;
    wm.getEntity("table")->shape();
    std::cout << "renderImage: done getting shape and entity" << std::endl;
    const ed::WorldModel* wm_ptr = &wm;
    ed::EntityConstPtr e_const_ptr = wm.getEntity("table");
    const ed::Entity* e_ptr = e_const_ptr.get();
    std::cout << "WM: " << wm_ptr << ", e: " << e_ptr << std::endl;
    std::cout << "" << std::endl;
    /// END OF TESTCODE
    bool result = ed::renderWorldModel(wm, ed::ShowVolumes::NoVolumes, rasterizer, cam_pose, depth_image, image);
    std::cout << "\nRender result: " << result << "\n" << std::endl;

    if (SHOW_DEBUG_IMAGES)
    {
        cv::namedWindow("Colored image", 1);
        cv::imshow("Colored image", image);
        std::cout << "Press any key to continue" << std::endl;
        cv::waitKey(0);
        cv::namedWindow("Depth image", 1);
        cv::imshow("Depth image", depth_image);
        std::cout << "Press any key to continue" << std::endl;
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

//    depth_image.setTo(0.0f);

//    // Iterate over all entities
//    std::cout << "Iterate over all entities" << std::endl;

//    for(ed::WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
//    {
//        std::cout << "\nNext entity... " << std::endl;
//        const ed::EntityConstPtr& e = *it;
//        std::cout << "Entity ID: " << e->id() << std::endl;
        
////        std::cout << "Getting shaperevision" << std::endl;
////        int shaperevision = e->shapeRevision();
////        std::cout << "ShapeRevision: " << shaperevision << std::endl;
////        if (shaperevision == 0)
////        {
////            std::cout << "Entity " << e->id() << " does not have a shape, skipping..." << std::endl;
////            continue;
////        }

////        std::cout << "Getting shape" << std::endl;
////        geo::ShapeConstPtr shape_ptr = e->shape();
////        std::cout << "\tType: " << shape_ptr->TYPE << std::endl;
////        std::cout << "\tShapeRevision: " << e->shapeRevision() << std::endl;
////        std::cout << "\tEmpty: " << shape_ptr->empty() << std::endl;
////        if (shape_ptr)
////            std::cout << "Entity " << e->id() << " has a shape of type " << shape_ptr->TYPE << " with shaperevision " << e->shapeRevision() << " and empty: " << shape_ptr->empty() << std::endl;
////        else
////        {
////            std::cout << "Entity " << e->id() << " does not have a shape" << std::endl;
////            continue;
////        }

//        // Render ...
////        std::cout << "Rendering entity " << e->id() << std::endl;
////        geo::Transform t(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
////        rasterizer.rasterize(*shape_ptr, t, depth_image);
//        std::cout << "Entity done" << std::endl;

//    }
//    std::cout << "Iteration done" << std::endl;


////    const std::map<std::string, Object*>& objects = world.getObjects();
////    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
////        Object* obj = it_obj->second;

////        geo::ShapePtr shape = obj->getShape();
////        if (shape) {
////            geo::Transform t = tf_map_to_kinect.inverse() * obj->getAbsolutePose();
////            rasterizer_.rasterize(*shape, t, depth_image);
////        }

////        std::vector<Object*> children;
////        obj->getChildrenRecursive(children);

////        for(std::vector<Object*>::const_iterator it = children.begin(); it != children.end(); ++it) {
////            const Object& child = **it;
////            geo::ShapePtr child_shape = child.getShape();
////            if (child_shape) {
////                geo::Transform t = tf_map_to_kinect.inverse() * child.getAbsolutePose();
////                rasterizer_.rasterize(*child_shape, t, depth_image);
////            }
////        }
////    }

}


void createWorldModel(ed::WorldModel& wm)
{
    // ToDos:
    // * fix paths in this package (won't work in CI this way since the table (and floor) cannot be used yet)

    // Load the table model from the sdf file
    std::string path = ros::package::getPath("ed_sensor_integration");
    path += "/test/test_model.sdf";
    ed::UpdateRequest request;
    ed::models::loadModel(ed::models::LoadType::FILE, path, request);

    wm.update(request);

}


void moveFurnitureObject(const ed::UUID& id, const geo::Pose3D& new_pose, ed::WorldModel& wm)
{
    ed::UpdateRequest request;
//    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
//    t1->insert(0, geo::Pose3D(x, y, z, roll, pitch, yaw));
//    request.setRelation("floor", "table", t1);
    request.setPose(id, new_pose);
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
    std::cout << "Fitting: Processing sensor data" << std::endl;
    fitter.processSensorData(*image, sensor_pose, fitterdata);

    std::cout << "Fitting: getting entity" << std::endl;
    ed::EntityConstPtr e = wm.getEntity(entity_id);
    e->shape();
//    std::cout << "Fitting: getting entity shape"
    // ToDo: what do we want to do if we cannot find the entity?
    if (!e)
        throw std::runtime_error("Entity not found in WM");
    std::cout << "Fitting: estimating entity pose" << std::endl;
    return fitter.estimateEntityPose(fitterdata, wm, entity_id, e->pose(), new_pose, max_yaw_change);
}


/**
 * @brief testSinglePose
 * @param rasterizer
 * @param cam_model
 * @param cam_pose
 * @param wm
 * @param new_pose
 * @param fitter
 * @param result N.B.: the result is stored here instead of return value due to the ASSERT
 */
void testSinglePose(const geo::DepthCamera& rasterizer,
                    const image_geometry::PinholeCameraModel& cam_model,
                    const geo::Pose3D& cam_pose,
                    const ed::WorldModel& wm,
                    const geo::Pose3D& new_pose,
                    Fitter& fitter,
                    bool& result)
{
    // Move the table
    ed::WorldModel wm_copy(wm);
//	geo::Pose3D new_pose(x, y, 0.0, 0.0, 0.0, yaw);
    moveFurnitureObject("table", new_pose, wm_copy);

    /// Check to see if the pose of the table in the original world model has not changed
    ed::EntityConstPtr table_entity = wm.getEntity("table");
    geo::Vec3 pos_diff = geo::Vec3(0.0, 0.0, 0.0) - table_entity->pose().t;
    ASSERT_TRUE(pos_diff.length() < 0.001);
    std::cout << "Pose of the table in the original WM: " << table_entity->pose() << std::endl;

    // Render another image
    cv::Mat depth_image2(CAM_RESOLUTION_HEIGHT, CAM_RESOLUTION_WIDTH, CV_32FC1, 0.0); // ToDo: check!
    renderImage(rasterizer, cam_pose, wm_copy, depth_image2);

    // Start fitting
    std::cout << "Creating RGBD image" << std::endl;
    rgbd::Image rgbd_image(depth_image2, // ToDo: replace by colored image
           depth_image2,
           cam_model,
           "camera", // ToDo: check if frame id is necessay
           0.0); // ToDo: check if valid stamp is necessary
    std::cout << "Instantiating 'new pose' " << std::endl;
    geo::Pose3D fitted_pose;
    std::cout << "Fitting supporting entity" << std::endl;
    //    bool fit_result = fitSupportingEntity(&rgbd_image, cam_pose,
    //                                          wm_copy, ed::UUID("table"), 45.0 / 180.0 * 3.14, // ToDo: nicer max yaw angle
    //                                          fitter, new_pose);
    // ToDo: use wm copy
    //    const std::string& furniture_id = "table";
    //    ed::UUID ed_furniture_id(furniture_id);
    ed::UUID ed_furniture_id("table");
    bool fit_result = fitSupportingEntity(&rgbd_image, cam_pose,
              wm, ed_furniture_id, 45.0 / 180.0 * M_PI, // ToDo: nicer max yaw angle
              fitter, fitted_pose);
    std::cout << "Fit result: " << fit_result <<
         "\nExpected pose: " << new_pose <<
         "\nComputed pose: " << fitted_pose <<
         std::endl;

    geo::Vec3 pos_error = new_pose.t - fitted_pose.t;
    double yaw_error = getYaw(new_pose.R) - getYaw(fitted_pose.R);
    // ToDo: add yaw error
    if (pos_error.length() > 0.05 || fabs(yaw_error) > 5.0 / 180.0 * M_PI) // ToDo: this is still quite a lot
    {
        result = false;
    } else
    {
        result = true;
    }
}


TEST(TestSuite, testCase)
{
//    for (unsigned int i = 0; i < 10000; i++)
//    {
//        ed::Entity e;
//        ASSERT_FALSE(e.shape());
//    }

    // Setup world model
    std::cout << "Starting testsuite" << std::endl;
    ed::WorldModel wm;
    createWorldModel(wm);

    // Create rasterizer that is used for rendering of depth images
    image_geometry::PinholeCameraModel cam_model;
    geo::DepthCamera rasterizer;
    setupRasterizer(cam_model, rasterizer);

    // Set up the fitter
    Fitter fitter;

    // Camera pose
    geo::Pose3D cam_pose;
    cam_pose.t = geo::Vector3(-1.5, 0.0, 1.5);
    cam_pose.setRPY(0.0, -1.57, 0.0);  // In view, rotated 90 degrees
    cam_pose.setRPY(1.57, 0.0, -1.57);  // In view, straight
    cam_pose.setRPY(0.87, 0.0, -1.57);  // In view, tilted at table

    // Render image
    cv::Mat depth_image(CAM_RESOLUTION_HEIGHT, CAM_RESOLUTION_WIDTH, CV_32FC1, 0.0); // ToDo: check!
    renderImage(rasterizer, cam_pose, wm, depth_image);

    bool test_result = true;
    unsigned int nr_tests = 0;
    std::vector<geo::Pose3D> failed_poses;

    for (double x = -0.5; x <= 0.5; x += 0.1)
    {
        for (double y = -0.5; y <= 0.5; y += 0.1)
//          double y = 0.0;
        {
            for (double yaw_deg = -40.0; yaw_deg <= 40.0; yaw_deg += 10.0)
//            double yaw_deg = 0.0;
            {
                ++nr_tests;
                double yaw = yaw_deg * M_PI / 180.0;

                geo::Pose3D new_pose(x, y, 0.0, 0.0, 0.0, yaw);
                bool single_res = false;
                testSinglePose(rasterizer, cam_model, cam_pose, wm, new_pose, fitter, single_res);
                if (!single_res)
                {
                    test_result = false;
                    failed_poses.push_back(new_pose);
                }

            } // end of yaw loop
        } // end of y loop
    } // end of x loop

    std::cout << "Tests done 1" << std::endl;
    std::cout << "Failed poses: (" << failed_poses.size() << " out of " << nr_tests << ")" << std::endl;
    for (auto& failed_pose: failed_poses)
    {
        std::cout << "\n" << failed_pose << std::endl;
    }
    ASSERT_TRUE(test_result);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

//  g_argc = argc;
//  g_argv = argv;

  // ToDo: get this from CLI args
//  SHOW_DEBUG_IMAGES = true;

  return RUN_ALL_TESTS();
}

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
        {
            for (double yaw_deg = -40.0; yaw_deg <= 40.0; yaw_deg += 10.0)
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

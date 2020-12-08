#include <gtest/gtest.h>
#include <math.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <image_geometry/pinhole_camera_model.h>
#include <ros/console.h>
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

const uint CAM_RESOLUTION_WIDTH = 640;
const uint CAM_RESOLUTION_HEIGHT = 480;

const double MAX_POSITION_ERROR = 0.05;
const double MAX_YAW_ERROR_DEGREES = 5.0;

// The following two constants are added for regression: not all poses
// will succeed. However, we don't want performance to get worse
const uint NR_TEST_POSES = 1089;
const uint NR_SUCCEEDED_POSES = 1089 - 106;

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
    ROS_DEBUG_STREAM("Matrix: " << rotation << " --> yaw: " << angles.yaw);
    return angles.yaw;
}


/**
 * @brief degToRad converts degrees to radians
 * @param input
 * @return
 */
double degToRad(double input)
{
    return input * M_PI / 180.0;
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
    ROS_DEBUG_STREAM("\nRender result: " << result << "\n");

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
    ROS_DEBUG_STREAM("Fitting: Processing sensor data");
    fitter.processSensorData(*image, sensor_pose, fitterdata);

    ROS_DEBUG_STREAM("Fitting: getting entity");
    ed::EntityConstPtr e = wm.getEntity(entity_id);
    if (!e)
        throw std::runtime_error("Entity not found in WM");
    ROS_DEBUG_STREAM("Fitting: estimating entity pose");
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
bool testSinglePose(const geo::DepthCamera& rasterizer,
                    const image_geometry::PinholeCameraModel& cam_model,
                    const geo::Pose3D& cam_pose,
                    const ed::WorldModel& wm,
                    const geo::Pose3D& new_pose,
                    Fitter& fitter)
{
    // Move the table
    ed::WorldModel wm_copy(wm);
    moveFurnitureObject("table", new_pose, wm_copy);

    /// Check to see if the pose of the table in the original world model has not changed
    ed::EntityConstPtr table_entity = wm.getEntity("table");
    geo::Vec3 pos_diff = geo::Vec3(0.0, 0.0, 0.0) - table_entity->pose().t;
    if (pos_diff.length() > 0.001)
        throw std::runtime_error("Table has moved in the original world model");
    ROS_DEBUG_STREAM("Pose of the table in the original WM: " << table_entity->pose());

    // Render another image
    cv::Mat depth_image2(CAM_RESOLUTION_HEIGHT, CAM_RESOLUTION_WIDTH, CV_32FC1, 0.0); // ToDo: check!
    renderImage(rasterizer, cam_pose, wm_copy, depth_image2);

    // Start fitting
    ROS_DEBUG_STREAM("Creating RGBD image");
    rgbd::Image rgbd_image(depth_image2, // ToDo: replace by colored image
           depth_image2,
           cam_model,
           "camera", // ToDo: check if frame id is necessay
           0.0); // ToDo: check if valid stamp is necessary
    ROS_DEBUG_STREAM("Instantiating 'new pose' ");
    geo::Pose3D fitted_pose;
    ROS_DEBUG_STREAM("Fitting supporting entity");
    ed::UUID ed_furniture_id("table");
    bool fit_result = fitSupportingEntity(&rgbd_image, cam_pose,
              wm, ed_furniture_id, degToRad(45.0), // ToDo: nicer max yaw angle
              fitter, fitted_pose);
    ROS_DEBUG_STREAM("Fit result: " << fit_result <<
         "\nExpected pose: " << new_pose <<
         "\nComputed pose: " << fitted_pose
         );

    geo::Vec3 pos_error = new_pose.t - fitted_pose.t;
    double yaw_error = getYaw(new_pose.R) - getYaw(fitted_pose.R);
    if (pos_error.length() > MAX_POSITION_ERROR || fabs(yaw_error) > degToRad(MAX_YAW_ERROR_DEGREES))
        return false;
    else
        return true;
}


class FurnitureFitTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        ROS_DEBUG_STREAM("Setting up... ");
        extendPath("ED_MODEL_PATH");
        extendPath("GAZEBO_MODEL_PATH");
        extendPath("GAZEBO_RESOURCE_PATH");
    }

    void extendPath(const std::string& name)
    {
        if (const char* original_path = ::getenv(name.c_str()))
            original_env_values_[name] = original_path;
        else
            original_env_values_[name] = nullptr;
        std::string package_path = ros::package::getPath("ed_sensor_integration");
        std::string new_path = package_path + "/test";
        if (original_env_values_[name])
        {
            new_path += ":";
            new_path += original_env_values_[name];
        }
        ROS_INFO_STREAM("New " << name << ": " << new_path);
        ::setenv(name.c_str(), new_path.c_str(), 1);
    }

    void TearDown() override
    {
        ROS_DEBUG_STREAM("Tearing down... ");
        for (auto it = original_env_values_.begin(); it != original_env_values_.end(); it++)
        {
            if(it->second)
                ::setenv(it->first.c_str(), it->second, 1);
            else
                ::unsetenv(it->first.c_str());
        }
    }

    std::map<std::string, const char*> original_env_values_;

};


TEST_F(FurnitureFitTest, testCase)
{

    // Setup world model
    ROS_INFO_STREAM("Starting testsuite");
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

    // Count the number of tested poses and store the failed ones for later inspection
    unsigned int nr_poses = 0;
    std::vector<geo::Pose3D> failed_poses;

    for (double x = -0.5; x <= 0.5; x += 0.1)
    {
        for (double y = -0.5; y <= 0.5; y += 0.1)
        {
            for (double yaw_deg = -40.0; yaw_deg <= 40.0; yaw_deg += 10.0)
            {
                ++nr_poses;
                geo::Pose3D new_pose(x, y, 0.0, 0.0, 0.0, degToRad(yaw_deg));
                if (!testSinglePose(rasterizer, cam_model, cam_pose, wm, new_pose, fitter))
                {
                    failed_poses.push_back(new_pose);
                }

            } // end of yaw loop
        } // end of y loop
    } // end of x loop

    uint nr_succeeded_poses = nr_poses - failed_poses.size();
    ROS_INFO_STREAM("Tested " << nr_poses << " table poses, succeeded: "
                    << nr_succeeded_poses << ", failed: " << failed_poses.size());
    for (auto& failed_pose: failed_poses)
    {
        ROS_DEBUG_STREAM("\n" << failed_pose);
    }
    ASSERT_TRUE(nr_succeeded_poses >= NR_SUCCEEDED_POSES);
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

#include <gtest/gtest.h>
#include <math.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/console.h>
#include <ros/package.h>

// TU/e Robotics
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/models/model_loader.h>
#include <ed/update_request.h>
#include <ed/rendering.h>
#include <ed/uuid.h>
#include <ed/relations/transform_cache.h>
#include <geolib/Shape.h>
#include "tue/config/reader_writer.h"
#include "tue/config/loaders/sdf.h"

// ED sensor integration
#include <ed/laser/laser_plugin.h>

const double MAX_POSITION_ERROR = 0.05;
const double MAX_YAW_ERROR_DEGREES = 5.0;

bool SHOW_DEBUG_IMAGES = false;


void moveFurnitureObject(const ed::UUID& id, const geo::Pose3D& new_pose, ed::WorldModel& wm)
{
    ed::UpdateRequest request;
    request.setPose(id, new_pose);
    wm.update(request);
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



TEST_F(LaserPluginUpdateTest, testCase)
{
    ROS_INFO_STREAM("Starting testsuite");

    ed::WorldModel world_model_;
    std::string path = ros::package::getPath("ed_sensor_integration");
    path += "/test/test_model.sdf";
    ed::UpdateRequest request;
    ed::models::loadModel(ed::models::LoadType::FILE, path, request);

    world_model_.update(request);

    // create laserscan message

    geo::Pose3D test_pose(-0.5, -0.5, 0.0, 0.0, 0.0, 0.0);
    ASSERT_TRUE(test_setup.testSinglePose(test_pose));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

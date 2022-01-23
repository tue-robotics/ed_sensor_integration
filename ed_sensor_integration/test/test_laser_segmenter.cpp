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
#include <ed/laser/updater.h>

const double MAX_POSITION_ERROR = 0.05;
const double MAX_YAW_ERROR_DEGREES = 5.0;

void moveFurnitureObject(const ed::UUID& id, const geo::Pose3D& new_pose, ed::WorldModel& wm)
{
    ed::UpdateRequest request;
    request.setPose(id, new_pose);
    wm.update(request);
}


class LaserSegmenterTest : public ::testing::Test {
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


TEST_F(LaserSegmenterTest, testCase)
{
    ROS_INFO_STREAM("Starting testsuite");

    std::string path = ros::package::getPath("ed_sensor_integration");
    path += "/test/test_laser_model.sdf";

    ed::UpdateRequest request;
    ed::models::loadModel(ed::models::LoadType::FILE, path, request);

    ed::WorldModel world_model;
    world_model.update(request);

    // create updater
    LaserUpdater updater;
    uint num_beams = 1000;
    updater.world_association_distance_ = 0.4;
    updater.min_segment_size_pixels_ = 5;
    updater.segment_depth_threshold_ = 0.2;
    updater.min_cluster_size_ = 0.1;
    updater.max_cluster_size_ = 3.0;
    updater.fit_entities_ = false;
    updater.max_gap_size_ = 10;
    updater.configureLaserModel(num_beams, -2.0, 2.0 , 0.01, 30.0);
    
    std::vector<double> sensor_ranges(num_beams, 0);
    geo::Pose3D test_pose(0.0, 0.0, 0.2, 0.0, 0.0, 0.0);

    updater.renderWorld(test_pose, world_model, sensor_ranges);

    ed::UpdateRequest req;
    double timestamp = 0.0;

    ed::WorldModel empty_world;
    updater.update(empty_world, sensor_ranges, test_pose, timestamp, req);

    int n_entities_found = req.updated_entities.size();
    EXPECT_EQ(n_entities_found, 3) << "3 entities in world, updater found " << n_entities_found;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <iostream>

// ROS
#include <ros/package.h>

// TU/e Robotics
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/models/model_loader.h>
#include <ed/update_request.h>
#include <ed/relations/transform_cache.h>
#include "tue/config/reader_writer.h"
#include "tue/config/loaders/sdf.h"


/**
 * @brief renderImage renders an rgbd image from a world model.
 * N.B.: copied from https://github.com/tue-robotics/fast_simulator/blob/master/src/Kinect.cpp
 * @param wm
 */
// ToDo: generalize
void renderImage(const ed::WorldModel& wm, cv::Mat& depth_image)
{
    depth_image.setTo(0.0f);

    // Iterate over all entities
//    for (auto it = wm.begin(); it != wm.end(); ++it)
//    for (ed::WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
//    {
//        std::cout << "Entity ID: " << it. << std::endl;
//    }
//    std::vector<ed::EntityConstPtr> entities = wm.entities();
//    for (auto it = entities.begin(); it != entities.end(); ++it)
//    {
//        std::cout << "Entity ID: " << it->get() << std::endl;
////        std::cout << "Entity ID: " << it->get().id() << std::endl;
//    }
//    for (const auto& entityptr: entities)
//    {
//        std::cout << "Entity ID: " << entityptr << std::endl;
////        std::cout << "Entity ID: " << it->get().id() << std::endl;
//    }
    for(ed::WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        std::cout << "Entity ID: " << e->id() << std::endl;
//	    if (!req.id.empty() && e->id() != ed::UUID(req.id))
//		continue;
    }


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


void createWorldModel(ed::WorldModel& wm)
{
    ed::models::ModelLoader loader;
    ed::UpdateRequest request;
    request.setType("map", "waypoint");
    request.setType("table", "object");

    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
    t1->insert(0, geo::Pose3D(0, 0, 0, 0, 0, 0));
    request.setRelation("map", "table", t1);

    // Load the table model from the sdf file
    std::string path = ros::package::getPath("ed_sensor_integration");
    path += "/test/table.sdf";
    tue::config::ReaderWriter config;
    tue::config::loadFromSDFFile(path, config);
    config.readGroup("sdf");

    std::stringstream errors;
    loader.create(config.data(), request, errors);

    wm.update(request);

}


TEST(TestSuite, testCase)
{
    std::cout << "Starting testsuite" << std::endl;
    ed::WorldModel wm;
    createWorldModel(wm);

    cv::Mat depth_image;
    renderImage(wm, depth_image);

    std::cout << "Tests done" << std::endl;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

//  g_argc = argc;
//  g_argv = argv;

  return RUN_ALL_TESTS();
}

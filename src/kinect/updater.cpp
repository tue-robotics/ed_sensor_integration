#include "ed/kinect/updater.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <rgbd/View.h>

#include <tue/config/reader.h>

#include <geolib/Shape.h>
#include <ed/serialization/serialization.h>

#include "ed/kinect/association.h"
#include "ed/kinect/renderer.h"

// ----------------------------------------------------------------------------------------------------

Updater::Updater()
{
}

// ----------------------------------------------------------------------------------------------------

Updater::~Updater()
{
}

// ----------------------------------------------------------------------------------------------------

bool Updater::update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                     const std::string& update_command, UpdateResult& res)
{
    // -------------------------------------
    // Parse space description (split on space)

    std::size_t i_space = update_command.find(' ');

    ed::UUID entity_id;
    std::string area_name;

    if (i_space == std::string::npos)
    {
        entity_id = update_command;
    }
    else
    {
        area_name = update_command.substr(0, i_space);
        entity_id = update_command.substr(i_space + 1);
    }

    // -------------------------------------
    // Check for entity and last image existence

    ed::EntityConstPtr e = world.getEntity(entity_id);

    if (!e)
    {
        res.error << "No such entity: '" << entity_id.str() << "'.";
        return false;
    }
    else if (!e->has_pose())
    {
        res.error << "Entity: '" << entity_id.str() << "' has no pose.";
        return false;
    }

    // -------------------------------------
    // Update entity position

    FitterData fitter_data;
    fitter_.processSensorData(*image, sensor_pose, fitter_data);

    geo::Pose3D new_pose;
    if (fitter_.estimateEntityPose(fitter_data, world, entity_id, e->pose(), new_pose))
    {
        res.update_req.setPose(entity_id, new_pose);
    }
    else
    {
        res.error << "Could not determine pose of '" << entity_id.str() << "'.";
        return false;
    }

    // -------------------------------------
    // Optimize sensor pose

    geo::Pose3D new_sensor_pose = sensor_pose;
//    fitZRP(*e->shape(), new_pose, image, sensor_pose, new_sensor_pose);

    std::cout << "Old sensor pose: " << sensor_pose << std::endl;
    std::cout << "New sensor pose: " << new_sensor_pose << std::endl;
    std::cout << std::endl;

    // -------------------------------------
    // Segment objects

    if (!area_name.empty())
    {
        // Determine segmentation area (the geometrical shape in which the segmentation should take place)

        geo::Shape shape;
        bool found = false;
        tue::config::Reader r(e->data());

        if (r.readArray("areas"))
        {
            while(r.nextArrayItem())
            {
                std::string a_name;
                if (!r.value("name", a_name) || a_name != area_name)
                    continue;

                if (ed::deserialize(r, "shape", shape))
                {
                    found = true;
                    break;
                }
            }

            r.endArray();
        }

        if (!found)
        {
            res.error << "No area '" << area_name << "' for entity '" << entity_id.str() << "'.";
            return false;
        }
        else if (shape.getMesh().getTriangleIs().empty())
        {
            res.error << "Could not load shape of area '" << area_name << "' for entity '" << entity_id.str() << "'.";
            return false;
        }

        geo::Pose3D shape_pose = new_sensor_pose.inverse() * new_pose;
        cv::Mat filtered_depth_image;
        segmenter_.calculatePointsWithin(*image, shape, shape_pose, filtered_depth_image);

        // Determine camera model
        rgbd::View view(*image, filtered_depth_image.cols);
        const geo::DepthCamera& cam_model = view.getRasterizer();

        segmenter_.cluster(filtered_depth_image, cam_model, sensor_pose, res.entity_updates);

        associateAndUpdate(world, image, sensor_pose, res.entity_updates, res.update_req);

    }

    return true;
}


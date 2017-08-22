#include "ed/kinect/updater.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <ed/helpers/depth_data_processing.h>

#include <rgbd/View.h>

#include <tue/config/reader.h>

#include <geolib/Shape.h>
#include <geolib/shapes.h>

#include <ed/serialization/serialization.h>

#include "ed/kinect/association.h"
#include "ed/kinect/renderer.h"

#include "ed/convex_hull_calc.h"

#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------

// Calculates which depth points are in the given convex hull (in the EntityUpdate), updates the mask,
// and updates the convex hull height based on the points found
void refitConvexHull(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const geo::DepthCamera& cam_model,
                     const Segmenter& segmenter_, EntityUpdate& up)
{
    up.pose_map.t.z += (up.chull.z_max + up.chull.z_min) / 2;

    geo::Shape chull_shape;
    std::vector<geo::Vec2> points(up.chull.points.size());
    for(unsigned int i = 0; i < points.size(); ++i)
        points[i] = geo::Vec2(up.chull.points[i].x, up.chull.points[i].y);
    geo::createConvexPolygon(chull_shape, points, up.chull.height());

    cv::Mat filtered_depth_image;
    segmenter_.calculatePointsWithin(image, chull_shape, sensor_pose.inverse() * up.pose_map, filtered_depth_image);

    up.points.clear();
    up.pixel_indices.clear();

    float z_min =  1e9;
    float z_max = -1e9;

    int i_pixel = 0;
    for(int y = 0; y < filtered_depth_image.rows; ++y)
    {
        for(int x = 0; x < filtered_depth_image.cols; ++x)
        {
            float d = filtered_depth_image.at<float>(y, x);
            if (d == 0)
            {
                ++i_pixel;
                continue;
            }

            geo::Vec3 p = cam_model.project2Dto3D(x, y) * d;
            geo::Vec3 p_map = sensor_pose * p;

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);

            up.pixel_indices.push_back(i_pixel);
            up.points.push_back(p);
            ++i_pixel;
        }
    }

    double h = z_max - z_min;
    up.pose_map.t.z = (z_max + z_min) / 2;
    up.chull.z_min = -h / 2;
    up.chull.z_max =  h / 2;
}

// ----------------------------------------------------------------------------------------------------

// Calculates which depth points are in the given convex hull (in the EntityUpdate), updates the mask,
// and updates the convex hull height based on the points found
std::vector<EntityUpdate> mergeOverlappingXYConvexHulls(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const geo::DepthCamera& cam_model,
                                                         const Segmenter& segmenter_, const std::vector<EntityUpdate>& updates)
{
  // Updated convex hulls
  std::vector<EntityUpdate> new_updates;

  // Keep track of indices that did collide, skip them in i
  std::vector<int> collided_indices;
  std::map<int, std::vector<int> > collission_map;

  for (int i = 0; i < updates.size(); ++i)
  {
    const EntityUpdate& u1 = updates[i];

    // If index already collided, it will be merged to another one
    if (std::find(collided_indices.begin(), collided_indices.end(), i) != collided_indices.end())
    {
      continue;
    }

    for (int j = 0; j < updates.size(); ++j)
    {
    // skip self
        if (i == j)
            continue;

      const EntityUpdate& u2 = updates[j];

      // If we collide, update the i convex hull
      if (ed::convex_hull::collide(u1.chull, u1.pose_map.t, u2.chull, u2.pose_map.t, 0, 1e6))
      {
        collission_map[i].push_back(j);
        collided_indices.push_back(j);
      }
    }
  }

  // Now again loop over the updates and only push back in the new updates if it did not collide
  for (int i = 0; i < updates.size(); ++i)
  {
    const EntityUpdate& u = updates[i];

    // If index already collided, it will be merged to another one
    if (std::find(collided_indices.begin(), collided_indices.end(), i) == collided_indices.end())
    {
      // Only push back if not in collided entity

      // TODO: merge the collided entity with this one; but how to merge, so much methods here .. forest and the trees
      new_updates.push_back(u);
    }
  }

  return new_updates;
}

// ----------------------------------------------------------------------------------------------------

Updater::Updater()
{
}

// ----------------------------------------------------------------------------------------------------

Updater::~Updater()
{
}

// ----------------------------------------------------------------------------------------------------

bool Updater::update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose_const,
                     const UpdateRequest& req, UpdateResult& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Prepare some things

    // will contain depth image filtered with given update shape and world model (background) subtraction
    cv::Mat filtered_depth_image;

    // sensor pose might be update, so copy (making non-const)
    geo::Pose3D sensor_pose = sensor_pose_const;

    // depth image
    const cv::Mat& depth = image->getDepthImage();

    // Determine depth image camera model
    rgbd::View view(*image, depth.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    std::string area_description;

    if (!req.area_description.empty())
    {
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if the update_command is a segmented entity.
        // If so, lookup the corresponding area_description

        bool fit_supporting_entity = true;

        std::map<ed::UUID, std::string>::const_iterator it_area_descr = id_to_area_description_.find(req.area_description);
        if (it_area_descr != id_to_area_description_.end())
        {
            area_description = it_area_descr->second;
            fit_supporting_entity = false; // We are only interested in the supported entity, so don't fit the supporting entity
        }
        else
            area_description = req.area_description;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Parse space description (split on space)

        std::size_t i_space = area_description.find(' ');

        ed::UUID entity_id;
        std::string area_name;

        if (i_space == std::string::npos)
        {
            entity_id = area_description;
        }
        else
        {
            area_name = area_description.substr(0, i_space);
            entity_id = area_description.substr(i_space + 1);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check for entity

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

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Update entity position

        geo::Pose3D new_pose;

        if (fit_supporting_entity)
        {
            FitterData fitter_data;
            fitter_.processSensorData(*image, sensor_pose, fitter_data);

            if (fitter_.estimateEntityPose(fitter_data, world, entity_id, e->pose(), new_pose, req.max_yaw_change))
            {
                res.update_req.setPose(entity_id, new_pose);
            }
            else
            {
//                res.error << "Could not determine pose of '" << entity_id.str() << "'.";
//                return false;

                // Could not fit entity, so keep the old pose
                new_pose = e->pose();
            }
        }
        else
        {
            new_pose = e->pose();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Optimize sensor pose

        if (false)
        {
            fitZRP(*e->shape(), new_pose, *image, sensor_pose_const, sensor_pose);

            std::cout << "Old sensor pose: " << sensor_pose_const << std::endl;
            std::cout << "New sensor pose: " << sensor_pose << std::endl;
            std::cout << std::endl;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Determine segmentation area

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

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Segment

            geo::Pose3D shape_pose = sensor_pose.inverse() * new_pose;
            segmenter_.calculatePointsWithin(*image, shape, shape_pose, filtered_depth_image);
        }
    }
    else
    {
        filtered_depth_image = image->getDepthImage().clone();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Remove background

    // The world model may have been updated above, but the changes are only captured in
    // an update request. Therefore, make a (shallow) copy of the world model and apply
    // the changes, and use this for the background removal

    ed::WorldModel world_updated = world;
    world_updated.update(res.update_req);

    segmenter_.removeBackground(filtered_depth_image, world_updated, cam_model, sensor_pose, req.background_padding);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear convex hulls that are no longer there

    std::vector<ed::EntityConstPtr> associatable_entities;
    for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
    {
        const ed::EntityConstPtr& e = *e_it;
        if (e->shape() || !e->has_pose() || e->convexHull().points.empty())
            continue;

        associatable_entities.push_back(e);
        /*
        geo::Vec3 p_3d = sensor_pose.inverse() * e->pose().t;

        cv::Point p_2d = cam_model.project3Dto2D(p_3d);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth.cols || p_2d.y >= depth.rows)
            continue;
        */
        /*float d = depth.at<float>(p_2d);
        if (d > 0 && d == d && -p_3d.z < d)
        {
            ROS_INFO("Request to remove entity %s", e->id().c_str());
            res.update_req.removeEntity(e->id());
            associatable_entities.pop_back();
        }*/
    }

//    cv::imshow("depth image", depth / 10);
//    cv::imshow("segments", filtered_depth_image / 10);
//    cv::waitKey();

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Cluster

    segmenter_.cluster(filtered_depth_image, cam_model, sensor_pose, res.entity_updates);

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Increase the convex hulls a bit towards the supporting surface and re-calculate mask
    // Then shrink the convex hulls again to get rid of the surface pixels

    for(std::vector<EntityUpdate>::iterator it = res.entity_updates.begin(); it != res.entity_updates.end(); ++it)
    {
        EntityUpdate& up = *it;

        up.chull.z_min -= 0.04;
        refitConvexHull(*image, sensor_pose, cam_model, segmenter_, up);

        up.chull.z_min += 0.01;
        refitConvexHull(*image, sensor_pose, cam_model, segmenter_, up);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Merge the detected clusters if they overlap in XY
    res.entity_updates = mergeOverlappingXYConvexHulls(*image, sensor_pose, cam_model, segmenter_, res.entity_updates);

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Perform association and update

    associateAndUpdate(associatable_entities, image, sensor_pose, res.entity_updates, res.update_req);

    // - - - - - - - - - - - - -  - - - - - - - -  - - -
    // Remove entities that are not associated
    for (std::vector<ed::EntityConstPtr>::const_iterator it = associatable_entities.begin(); it != associatable_entities.end(); ++it)
    {
        ed::EntityConstPtr e = *it;

        // Check if entity is in frustum
        geo::Vec3 p_3d = sensor_pose.inverse() * e->pose().t;
        cv::Point p_2d = cam_model.project3Dto2D(p_3d);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= depth.cols || p_2d.y >= depth.rows)
            continue;

        // If the entity is not updated, remove it
        if (res.update_req.updated_entities.find(e->id()) == res.update_req.updated_entities.end())
        {
            ROS_INFO("Entity not associated and not found in frustum");

            float d = depth.at<float>(p_2d);
            if (d > 0 && d == d && -p_3d.z < d)
            {
                ROS_INFO("We can shoot a ray through the center, removing entity %s", e->id().c_str());
                res.update_req.removeEntity(e->id());
            }
        }

    }

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Remember the area description with which the segments where found

    if (!area_description.empty())
    {
       for(std::vector<EntityUpdate>::const_iterator it = res.entity_updates.begin(); it != res.entity_updates.end(); ++it)
        {
            const EntityUpdate& up = *it;
            id_to_area_description_[up.id] = area_description;
        }
    }

    return true;
}

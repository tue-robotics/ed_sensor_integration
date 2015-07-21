#include "gui.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/entity.h>
#include <ed/world_model.h>

#include <geolib/ros/msg_conversions.h>

#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

class SimpleRenderResult : public geo::RenderResult
{

public:

    SimpleRenderResult(cv::Mat& z_buffer_, cv::Mat& entity_index_map_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_),
          entity_index_map(entity_index_map_), in_view(false) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
            entity_index_map.at<int>(y, x) = i_entity;
            in_view = true;
        }
    }

    cv::Mat& z_buffer;
    int i_entity;
    cv::Mat& entity_index_map;
    bool in_view;

};

// ----------------------------------------------------------------------------------------------------

void DrawWorldModelOverlay(const ed::WorldModel& world, const std::set<ed::UUID>& ids,
                           const std::set<ed::UUID>& updated_ids, Snapshot& snapshot,
                           bool force_change, bool& changed)
{
    int image_width = snapshot.background_image.cols;
    int image_height = snapshot.background_image.rows;

    geo::Pose3D sensor_pose = snapshot.sensor_pose_xya * snapshot.sensor_pose_zrp;

    rgbd::View view(*snapshot.image, image_width);

    cv::Mat depth_render(image_height, image_width, CV_32FC1, 0.0);
    cv::Mat entity_index_map(image_height, image_width, CV_32SC1, cv::Scalar(-1));

    SimpleRenderResult res(depth_render, entity_index_map);

    changed = false;
    int i_entity = 0;

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->shape() || !e->has_pose())
            continue;

        geo::RenderOptions opt;
        opt.setMesh(e->shape()->getMesh(), sensor_pose.inverse() * e->pose());

        if (ids.find(e->id()) != ids.end())
            res.i_entity = i_entity++;
        else
            res.i_entity = -1;

        res.in_view = false;
        view.getRasterizer().render(opt, res);

        if (res.i_entity >= 0)
        {
            // If the entity is in view, and the position was updated, the image changed
            if (res.in_view && updated_ids.find(e->id()) != updated_ids.end())
            {
                snapshot.visualized_ids.insert(e->id());
                changed = true;
            }

            // If the entity is not in view, but is was, the image also changed
            if (!res.in_view && snapshot.visualized_ids.find(e->id()) != snapshot.visualized_ids.end())
            {
                snapshot.visualized_ids.erase(e->id());
                changed = true;
            }
        }
    }

    // Check if there were entities in the snapshot that are now removed
    for(std::set<ed::UUID>::const_iterator it = updated_ids.begin(); it != updated_ids.end(); ++it)
    {
        const ed::UUID& id = *it;
        if (snapshot.visualized_ids.find(id) != snapshot.visualized_ids.end() && !world.getEntity(id))
        {
            // Entity was removed from view, so it changed
            changed = true;
        }
    }

    if (!changed && !force_change)
    {
        if (!snapshot.canvas.data)
            snapshot.canvas = snapshot.background_image;

        return;
    }

    // Draw world model on top of background
    snapshot.canvas = snapshot.background_image.clone();

    for(int y = 0; y < snapshot.canvas.rows - 1; ++y)
    {
        for(int x = 0; x < snapshot.canvas.cols - 1; ++x)
        {
            int i1 = entity_index_map.at<int>(y, x);
            int i2 = entity_index_map.at<int>(y, x + 1);
            int i3 = entity_index_map.at<int>(y + 1, x);

            if (i1 != i2 || i1 != i3)
            {
                // Entity edge
                snapshot.canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
            else if (i1 >= 0)
            {
                // Entity surface
                cv::Vec3b& c = snapshot.canvas.at<cv::Vec3b>(y, x);
                c[0] = std::min(255, 2 * c[0]);
                c[1] = 100;
                c[2] = 100;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Navigator::initialize(ros::NodeHandle& nh, const std::string& goal_topic)
{
    pub_goal_ = nh.advertise<cb_planner_msgs_srvs::LocalPlannerActionGoal>(goal_topic, 1);
}

// ----------------------------------------------------------------------------------------------------

bool Navigator::navigate(const Snapshot& snapshot, int click_x, int click_y)
{
    std::cout << "Navigate to pixel: " << click_x << ", " << click_y << std::endl;

    const cv::Mat& depth = snapshot.image->getDepthImage();

    if (click_x < 0 || click_x >= depth.cols || click_y < 0 || click_y >= depth.rows)
    {
        return false;
    }

    float d = depth.at<float>(click_y, click_x);
    for(int i = 1; i < 20; ++i)
    {
        for(int x = click_x - i; x <= click_x + i; ++x)
        {
            for(int y = click_y - i; y <= click_y + i; ++ y)
            {
                if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
                    continue;

                d = depth.at<float>(y, x);
                if (d > 0 && d == d)
                {
                    break;
                }
            }
            if (d > 0 && d == d)
                break;
        }
        if (d > 0 && d == d)
            break;
    }

    if (d <= 0 || d != d)
    {
        return false;
    }

    rgbd::View view(*snapshot.image, depth.cols);
    geo::Vec3 p_SENSOR = view.getRasterizer().project2Dto3D(click_x, click_y) * d;

    geo::Pose3D sensor_pose = (snapshot.sensor_pose_xya * snapshot.sensor_pose_zrp);

    geo::Vec3 p_MAP = sensor_pose * p_SENSOR;

    // Determine navigation goal
    geo::Vec3 robot_pos = sensor_pose.t;
    robot_pos.z = 0;

    double wanted_dist_to_goal = 2.0;

    p_MAP.z = 0;
    geo::Vec3 lookat_dir = (p_MAP - robot_pos).normalized();
    geo::Vec3 goal_pos = (p_MAP - wanted_dist_to_goal * lookat_dir);

    double dist_to_goal = (goal_pos - robot_pos).length();
    geo::Vec3 goal_dir = (goal_pos - robot_pos) / dist_to_goal;

    // Construct orientation matrix from direction
    geo::Mat3 ori = geo::Mat3::identity();
    ori.xx = lookat_dir.x;
    ori.yx = lookat_dir.y;
    ori.xy = -lookat_dir.y;
    ori.yy = lookat_dir.x;

    std::vector<geo::Pose3D> path;
    path.push_back(geo::Pose3D(ori, robot_pos));

    double waypoint_dist = 0.1;
    int num_waypoints = dist_to_goal / waypoint_dist;

    for(int i = 0; i < num_waypoints; ++i)
    {
        geo::Pose3D& p = path.back();
        path.push_back(geo::Pose3D(ori, p.t + waypoint_dist * goal_dir));
    }
    path.push_back(geo::Pose3D(ori, goal_pos));

    // -----------------------------------------------------------
    // Fill goal message

    // orientation
    cb_planner_msgs_srvs::LocalPlannerActionGoal goal_msg;
    goal_msg.goal.orientation_constraint.frame = "/map";
    geo::convert(p_MAP, goal_msg.goal.orientation_constraint.look_at);

    // path
    goal_msg.goal.plan.resize(path.size());
    for(unsigned int i = 0; i < path.size(); ++i)
    {
        geo::convert(path[i], goal_msg.goal.plan[i].pose);
        goal_msg.goal.plan[i].header.frame_id = "/map";
    }

    // -----------------------------------------------------------
    // Publish goal

    pub_goal_.publish(goal_msg);

    return true;
}



#include "gui.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/entity.h>
#include <ed/world_model.h>

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
                           const std::set<ed::UUID>& updated_ids, Snapshot& snapshot, bool& changed)
{
    geo::Pose3D sensor_pose = snapshot.sensor_pose_xya * snapshot.sensor_pose_zrp;

    const cv::Mat& original = snapshot.image->getRGBImage();

    rgbd::View view(*snapshot.image, original.cols);

    cv::Mat depth_render(original.rows, original.cols, CV_32FC1, 0.0);
    cv::Mat entity_index_map(original.rows, original.cols, CV_32SC1, cv::Scalar(-1));

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

    if (!changed)
        return;

    // Convert depth image to rgb
    snapshot.canvas = original.clone();

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


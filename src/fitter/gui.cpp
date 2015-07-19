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

void DrawWorldModelOverlay(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const ed::WorldModel& world,
                           const std::set<ed::UUID>& ids, cv::Mat& canvas, bool& changed)
{
    const cv::Mat& depth = image.getDepthImage();

    rgbd::View view(image, depth.cols);

    cv::Mat depth_render(depth.rows, depth.cols, CV_32FC1, 0.0);
    cv::Mat entity_index_map(depth.rows, depth.cols, CV_32SC1, cv::Scalar(-1));

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

        if (res.i_entity >= 0 && res.in_view)
            changed = true;
    }

    if (ids.empty())     // Simple hack to ensure that if there are no fitted entities left,
                         // images are still updated. (TODO: nicer fix)
        changed = true;

    if (!changed)
        return;

    // Convert depth image to rgb
    canvas = cv::Mat(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for(unsigned int i = 0; i < depth.rows * depth.cols; ++i)
        canvas.at<cv::Vec3b>(i) = (depth.at<float>(i) / 10) * cv::Vec3b(255, 255, 255);

    for(int y = 0; y < canvas.rows - 1; ++y)
    {
        for(int x = 0; x < canvas.cols - 1; ++x)
        {
            int i1 = entity_index_map.at<int>(y, x);
            int i2 = entity_index_map.at<int>(y, x + 1);
            int i3 = entity_index_map.at<int>(y + 1, x);

            if (i1 != i2 || i1 != i3)
            {
                // Entity edge
                canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
            else if (i1 >= 0)
            {
                // Entity surface
                cv::Vec3b& c = canvas.at<cv::Vec3b>(y, x);
                c[0] = std::min(255, 2 * c[0]);
                c[1] = 100;
                c[2] = 100;
            }
        }
    }
}


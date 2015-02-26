#include "sampling_projector_localizer.h"

#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

namespace
{

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), in_view(false) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0)
        {
            pixels.push_back(geo::Vec2u(x, y));
            in_view = true;
            z_buffer.at<float>(y, x) = depth;
        }
        else if (depth < old_depth)
        {
            in_view = true;
            z_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat& z_buffer;
    bool in_view;
    std::vector<geo::Vec2u> pixels;

};

}

// ----------------------------------------------------------------------------------------------------

SamplingProjectorLocalizer::SamplingProjectorLocalizer()
{
}

// ----------------------------------------------------------------------------------------------------

SamplingProjectorLocalizer::~SamplingProjectorLocalizer()
{
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D SamplingProjectorLocalizer::localize(const geo::Pose3D& sensor_pose, const rgbd::Image& image, const ed::WorldModel& world, const std::set<ed::UUID>& loc_ids)
{
    // - - - - - - - - - - - - - - - - - -
    // Render world model based on pose calculated above

    rgbd::View low_view(image, 80);
    const geo::DepthCamera& low_rasterizer = low_view.getRasterizer();

    const cv::Mat& depth_image = image.getDepthImage();
    rgbd::View full_view(image, depth_image.cols);
    const geo::DepthCamera& full_rasterizer = full_view.getRasterizer();


    cv::Mat model(low_view.getHeight(), low_view.getWidth(), CV_32FC1, 0.0);
    SampleRenderResult res(model);

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && (loc_ids.empty() || loc_ids.find(e->id()) != loc_ids.end()))
        {
            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            // Render
            low_rasterizer.render(opt, res);

            if (res.in_view)
                entities.push_back(e);
        }
    }

    std::cout << res.pixels.size() << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Try other poses and determine best scoring pose

    double min_error = 1e9; // TODO
    geo::Pose3D best_pose;

    std::vector<geo::Vector3> points(res.pixels.size());
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        const geo::Vec2u& p = res.pixels[i];
        points[i] = low_rasterizer.project2Dto3D(p.x, p.y) * model.at<float>(p.y, p.x);
    }

    for(double dx = -0.2; dx < 0.2; dx += 0.05)
    {
        for(double dy = -0.2; dy < 0.2; dy += 0.05)
        {
            for(double da = -0.1; da < 0.1; da += 0.05)
            {
                geo::Matrix3 m;
                m.setRPY(0, 0, da);

                geo::Pose3D test_pose;
                test_pose.t = sensor_pose.t + geo::Vector3(dx, dy, 0);
                test_pose.R = m * sensor_pose.R;

                geo::Pose3D T = (sensor_pose.inverse() * test_pose).inverse();
                geo::Pose3D T2 = T;

                T2.R.xx *= full_rasterizer.getFocalLengthX();
                T2.R.xy *= full_rasterizer.getFocalLengthX();
                T2.R.xz *= full_rasterizer.getFocalLengthX();
                T2.t.x  *= full_rasterizer.getFocalLengthX();

                T2.R.yx *= -full_rasterizer.getFocalLengthY();
                T2.R.yy *= -full_rasterizer.getFocalLengthY();
                T2.R.yz *= -full_rasterizer.getFocalLengthY();
                T2.t.y  *= -full_rasterizer.getFocalLengthY();

                double total_error = 0;
                int n = 0;

                for(unsigned int i = 0; i < res.pixels.size(); ++i)
                {
                    const geo::Vector3& p3d = points[i];

                    geo::Vector3 p_new = T2 * p3d;
                    p_new.x = p_new.x / -p_new.z + full_rasterizer.getOpticalCenterX();
                    p_new.y = p_new.y / -p_new.z + full_rasterizer.getOpticalCenterY();

                    if (p_new.x >= 0 && p_new.y >= 0 && p_new.x < depth_image.cols && p_new.y < depth_image.rows)
                    {
                        float ds = depth_image.at<float>(p_new.y, p_new.x);

                        if (ds > 0) // TODO
                        {
                            float dm = -p_new.z;
                            float err = std::min<float>(0.05, std::abs(dm - ds));
                            total_error += (err * err);
                            ++n;
                        }

                    }
                }

                if (n > 0)
                {
                    double avg_error = total_error / n;
                    if (avg_error < min_error)
                    {
                        min_error = avg_error;
                        best_pose = test_pose;
                    }
                }
            }
        }
    }

    return best_pose;
}


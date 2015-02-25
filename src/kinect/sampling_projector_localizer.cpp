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

    rgbd::View view(image, 160);

    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
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
            rasterizer.render(opt, res);

            if (res.in_view)
                entities.push_back(e);
        }
    }

    std::cout << res.pixels.size() << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Try other poses and determine best scoring pose

    double min_error = 1e9; // TODO
    geo::Pose3D best_pose;

    for(double dx = -0.2; dx < 0.2; dx += 0.05)
    {
        for(double dy = -0.2; dy < 0.2; dy += 0.05)
        {
            for(double da = -0.1; da < 0.1; da += 0.05)
            {

//    for(double dx = -0.2; dx < 0.2; dx += 0.025)
//    {
//        for(double dy = -0.2; dy < 0.2; dy += 0.025)
//        {
//            for(double da = -0.05; da < 0.05; da += 0.025)
//            {
                geo::Matrix3 m;
                m.setRPY(0, 0, da);

                geo::Pose3D test_pose;
                test_pose.t = sensor_pose.t + geo::Vector3(dx, dy, 0);
                test_pose.R = m * sensor_pose.R;

                geo::Pose3D test_pose_delta = (sensor_pose.inverse() * test_pose).inverse();

                double total_error = 0;
                int n = 0;

                for(unsigned int i = 0; i < res.pixels.size(); ++i)
                {
                    const geo::Vec2u& p = res.pixels[i];
                    geo::Vector3 p3d = rasterizer.project2Dto3D(p.x, p.y) * model.at<float>(p.y, p.x);

                    geo::Vector3 p3d_new = test_pose_delta * p3d;

                    cv::Point2d p_new = rasterizer.project3Dto2D(p3d_new);

                    if (p_new.x >= 0 && p_new.y >= 0 && p_new.x < view.getWidth() && p_new.y < view.getHeight())
                    {
                        float dm = -p3d_new.z;
                        float ds = view.getDepth(p_new.x, p_new.y);

                        if (dm > 0 && ds > 0) // TODO
                        {
                            if (std::abs(dm - ds) > 0.05)
                                total_error += 1;
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

//    std::cout << "Got here" << std::endl;

//    cv::imshow("test", best_model / 8);
//    cv::waitKey(3);

//    std::cout << "and here" << std::endl;


    // Render world
    cv::Mat best_model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
    SampleRenderResult res2(best_model);

    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        geo::Pose3D pose = best_pose.inverse() * e->pose();
        geo::RenderOptions opt;
        opt.setMesh(e->shape()->getMesh(), pose);

        // Render
        rasterizer.render(opt, res2);
    }

    // - - - - - - - - - - - - - - - - - -
    // Calculate and show diff

    cv::Mat diff(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);

    for(int y = 0; y < view.getHeight(); ++y)
    {
        for(int x = 0; x < view.getWidth(); ++x)
        {
            float dm = best_model.at<float>(y, x);
            float ds = view.getDepth(x, y);

            if (dm > 0 && ds > 0)
            {
                float err = std::abs(dm - ds);
                if (err > 0.05)
                    diff.at<float>(y, x) = err;
            }
        }
    }

    cv::imshow("depth", image.getDepthImage() / 8);
    cv::imshow("initial model", model / 8);
    cv::imshow("best model", best_model / 8);
    cv::imshow("diff", diff);
    cv::waitKey(3);

    return best_pose;
}


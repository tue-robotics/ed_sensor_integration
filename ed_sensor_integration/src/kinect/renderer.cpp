#include "ed/kinect/renderer.h"

#include <ed/entity.h>
#include <rgbd/view.h>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

namespace
{

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer, std::vector<unsigned int>& pixels)
        : geo::RenderResult(z_buffer.cols, z_buffer.rows), z_buffer_(z_buffer), pixels_(pixels)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0)
        {
            z_buffer_.at<float>(y, x) = depth;
            pixels_.push_back(y * z_buffer_.cols + x);
        }
        else if (depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;
        }
    }

protected:
    cv::Mat& z_buffer_; // Depth image
    std::vector<unsigned int>& pixels_; // Maps index of occupied pixels to actual pixel position

};

}

// ----------------------------------------------------------------------------------------------------

void fitZRP(const geo::Shape& shape, const geo::Pose3D& shape_pose, const rgbd::Image& image,
            const geo::Pose3D& sensor_pose, geo::Pose3D& updated_sensor_pose)
{
    rgbd::View view(image, 80);

    const geo::DepthCamera& rasterizer = view.getRasterizer();

    unsigned int width = view.getWidth();
    unsigned int height = view.getHeight();

    std::vector<unsigned int> pixels;
    pixels.reserve(width * height);

    // Create a resized version of the sensor depth image
    cv::Mat sensor_depth_img(height, width, CV_32FC1, 0.0);
    for (uint y = 0; y < height; ++y)
        for (uint x = 0; x < width; ++x)
            sensor_depth_img.at<float>(y, x) = view.getDepth(x, y);

    double min_error = 1e9; // TODO

    for(double droll = -0.0; droll <= 0.0; droll += 0.005)
    {
        for(double dpitch = -0.0; dpitch <= 0.0; dpitch += 0.005)
        {
            geo::Matrix3 m;
            m.setRPY(droll, dpitch, 0);

            geo::Pose3D test_pose;
            test_pose.t = sensor_pose.t;
            test_pose.R = m * sensor_pose.R;

            for(double dz = -0.05; dz < 0.05; dz += 0.005)
            {
                test_pose.t.z = sensor_pose.t.z + dz;

                // Render shape, given the test_pose as sensor_pose
                cv::Mat model(height, width, CV_32FC1, 0.0);
                SampleRenderResult res(model, pixels);
                geo::RenderOptions opt;
                opt.setMesh(shape.getMesh(), test_pose.inverse() * shape_pose);
                rasterizer.render(opt, res);

//                cv::imshow("sample", model / 10);
//                cv::waitKey(3);

                int n = 0;
                double total_error = 0;
                for(std::vector<unsigned int>::const_iterator it = pixels.cbegin(); it != pixels.cend(); ++it)
                {
                    unsigned int p_idx = *it;

                    float ds = sensor_depth_img.at<float>(p_idx);

                    if (ds > 0)
                    {
                        float dm = model.at<float>(p_idx);
                        float diff = std::abs(dm - ds);
                        if (diff > 0.01)
                            total_error += 0.1;
                        else
                            total_error += diff * diff;
//                        float err = std::min<float>(0.01, diff);
//                        total_error += (err * err);
                        ++n;
                    }
                }

                if (n > 0)
                {
                    double avg_error = total_error / n;
                    if (avg_error < min_error)
                    {
                        min_error = avg_error;
                        updated_sensor_pose = test_pose;
                    }
                }

            } // sample z
        } // sample pitch
    } // sample roll

}

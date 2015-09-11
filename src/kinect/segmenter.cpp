#include "segmenter.h"

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

class MinMaxRenderer : public geo::RenderResult
{

public:

    MinMaxRenderer(int width, int height) : geo::RenderResult(width, height)
    {
        min_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
        max_buffer = cv::Mat(height, width, CV_32FC1, 0.0);
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        // TODO: now the renderer can only deal with convex meshes, which means
        // that at each pixel there can only be one minimum and one maximum pixel
        // There is an easy solution for concave meshes: determine which side
        // the triangle points (away or to the camera) and test the pixel in the depth
        // image to be on the correct side. ... etc ...

        float d_min = min_buffer.at<float>(y, x);
        if (d_min == 0)
        {
            min_buffer.at<float>(y, x) = depth;
        }
        else if (depth < d_min)
        {
            min_buffer.at<float>(y, x) = depth;
            max_buffer.at<float>(y, x) = d_min;
        }
        else
        {
            max_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat min_buffer;
    cv::Mat max_buffer;

};

// ----------------------------------------------------------------------------------------------------

Segmenter::Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

Segmenter::~Segmenter()
{
}

// ----------------------------------------------------------------------------------------------------

void Segmenter::calculatePointsWithin(const rgbd::Image& image, const geo::Shape& shape,
                                      const geo::Pose3D& shape_pose, cv::Mat& filtered_depth_image)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render shape

    const cv::Mat& depth_image = image.getDepthImage();

    rgbd::View view(image, depth_image.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    MinMaxRenderer res(depth_image.cols, depth_image.rows);

    geo::RenderOptions opt;
    opt.setBackFaceCulling(false);
    opt.setMesh(shape.getMesh(), shape_pose);

    cam_model.render(opt, res);

    filtered_depth_image = cv::Mat(depth_image.rows, depth_image.cols, CV_32FC1, 0.0);

    for(int i = 0; i < depth_image.cols * depth_image.rows; ++i)
    {
        float d = depth_image.at<float>(i);
        if (d <= 0)
            continue;

        float d_min = res.min_buffer.at<float>(i);
        float d_max = res.max_buffer.at<float>(i);

        if (d_min > 0 && d_max > 0 && d > d_min && d < d_max)
            filtered_depth_image.at<float>(i) = d;
    }

    cv::imshow("min", res.min_buffer / 10);
    cv::imshow("max", res.max_buffer / 10);
    cv::imshow("filtered", filtered_depth_image / 10);
    cv::waitKey(3);
}

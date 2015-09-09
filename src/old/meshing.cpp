#include "ed_sensor_integration/meshing.h"

#include <opencv2/core/core.hpp>

#include <geolib/Shape.h>
#include <geolib/sensors/DepthCamera.h>

#include <opencv2/highgui/highgui.hpp>

namespace ed_sensor_integration
{

typedef std::vector<std::vector<int> > IndexMap;
typedef std::vector<std::vector<geo::Vector3> > VectorMap;

// -------------------------------------------------------------------------------

void addTriangle(const cv::Mat& depth_image, const geo::DepthCamera& rasterizer,
         int x1, int y1, int x2, int y2, int x3, int y3, geo::Mesh& mesh, IndexMap& vertex_index_map)
{

    float d1 = depth_image.at<float>(y1, x1);
    float d2 = depth_image.at<float>(y2, x2);
    float d3 = depth_image.at<float>(y3, x3);

    if (d1 == 0 || d2 == 0 || d3 == 0) {
        return;
    }

    int i1 = vertex_index_map[x1][y1];
    if (i1 < 0) {
        geo::Vector3 p = rasterizer.project2Dto3D(x1, y1) * d1;
        i1 = mesh.addPoint(p);
        vertex_index_map[x1][y1] = i1;
    }

    int i2 = vertex_index_map[x2][y2];
    if (i2 < 0) {
        geo::Vector3 p = rasterizer.project2Dto3D(x2, y2) * d2;
        i2 = mesh.addPoint(p);
        vertex_index_map[x2][y2] = i2;
    }

    int i3 = vertex_index_map[x3][y3];
    if (i3 < 0) {
        geo::Vector3 p = rasterizer.project2Dto3D(x3, y3) * d3;
        i3 = mesh.addPoint(p);
        vertex_index_map[x3][y3] = i3;
    }

    mesh.addTriangle(i1, i2, i3);
}

// -------------------------------------------------------------------------------

bool checkLineHorizontal(const cv::Mat& depth_image, int y, int x_start, int x_end, double error_threshold)
{
    float d_start = depth_image.at<float>(y, x_start);
    float d_end = depth_image.at<float>(y, x_end);

    if (d_start <= 0 || d_end <= 0) {
        return false;
    }

    float d_start_inv = 1 / d_start;
    float d_end_inv = 1 / d_end;

    int x_diff = x_end - x_start;
    float d_diff = d_end_inv - d_start_inv;

    float factor = 0.0f;
    float factorStep = 1.0f / (float)x_diff;

    int n = 0;
    double error = 0;
    for(int x = x_start; x <= x_end; ++x) {
        float d = depth_image.at<float>(y, x);
        if (d > 0) {
            float d_interpolated = 1 / (d_start_inv + (d_diff * factor));

            float diff = d - d_interpolated;
            error += diff * diff;
            ++n;

            if (error / n > error_threshold) {
                return false;
            }
        }

        factor += factorStep;
    }

    return true;
}

// -------------------------------------------------------------------------------

bool checkLineVertical(const cv::Mat& depth_image, int x, int y_start, int y_end, double error_threshold)
{
    float d_start = depth_image.at<float>(y_start, x);
    float d_end = depth_image.at<float>(y_end, x);

    if (d_start <= 0 || d_end <= 0) {
        return false;
    }

    float d_start_inv = 1 / d_start;
    float d_end_inv = 1 / d_end;

    int y_diff = y_end - y_start;
    float d_diff = d_end_inv - d_start_inv;

    float factorStep = 1.0f / (float)y_diff;
    float factor = factorStep;

    int n = 0;
    double error = 0;
    for(int y = y_start + 1; y < y_end; ++y) {
        float d = depth_image.at<float>(y, x);
        if (d > 0) {
            float d_interpolated = 1 / (d_start_inv + (d_diff * factor));

            float diff = d - d_interpolated;
            error += diff * diff;
            ++n;

            if (error / n > error_threshold) {
                return false;
            }
        }

        factor += factorStep;
    }

    return true;
}

// -------------------------------------------------------------------------------

bool checkBlock(const cv::Mat& depth_image, int x_start, int x_end, int y_start, int y_end, double error_threshold)
{
    for(int x = x_start; x <= x_end; ++x)
    {
        if (!checkLineVertical(depth_image, x, y_start, y_end, error_threshold))
            return false;
    }

    for(int y = y_start; y <= y_end; ++y)
    {
        if (!checkLineHorizontal(depth_image, y, x_start, x_end, error_threshold))
            return false;
    }

    return true;
}

// -------------------------------------------------------------------------------

void depthImageToMesh(const cv::Mat& depth_image, const geo::DepthCamera& rasterizer,int x_start, int x_end, int y_start, int y_end,
                                    double error_threshold, IndexMap& vertex_index_map, geo::Mesh& mesh)
{

    if (x_end <= x_start + 1 || y_end <= y_start + 1)
        return;

    if (checkBlock(depth_image, x_start, x_end, y_start, y_end, error_threshold))
    {
        addTriangle(depth_image, rasterizer, x_end, y_start, x_start, y_start, x_start, y_end, mesh, vertex_index_map);
        addTriangle(depth_image, rasterizer, x_end, y_start, x_start, y_end , x_end, y_end, mesh, vertex_index_map);
    }
    else
    {
        int x_half = (x_start + x_end) / 2;
        int y_half = (y_start + y_end) / 2;

        depthImageToMesh(depth_image, rasterizer, x_start, x_half, y_start, y_half, error_threshold, vertex_index_map, mesh);
        depthImageToMesh(depth_image, rasterizer, x_half, x_end,       y_start, y_half, error_threshold, vertex_index_map, mesh);

        depthImageToMesh(depth_image, rasterizer, x_start, x_half, y_half, y_end, error_threshold, vertex_index_map, mesh);
        depthImageToMesh(depth_image, rasterizer, x_half, x_end,       y_half, y_end, error_threshold, vertex_index_map, mesh);
    }
}

// --------------------------------------------------------------------------------

void depthImageToMesh(const cv::Mat& input, const geo::DepthCamera& rasterizer, double error_threshold, geo::Mesh& mesh)
{
    IndexMap vertex_index_map(input.cols, std::vector<int>(input.rows, -1));
    depthImageToMesh(input, rasterizer, 0, input.cols - 1, 0, input.rows - 1, error_threshold, vertex_index_map, mesh);
}

}

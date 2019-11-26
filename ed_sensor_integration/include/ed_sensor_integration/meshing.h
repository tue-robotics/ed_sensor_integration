#ifndef ED_SENSOR_INTEGRATION_MESHING_H_
#define ED_SENSOR_INTEGRATION_MESHING_H_

namespace cv
{
    class Mat;
}

namespace geo
{
    class DepthCamera;
    class Mesh;
}

#include <geolib/datatypes.h>

namespace ed_sensor_integration
{

    void depthImageToMesh(const cv::Mat& input, const geo::DepthCamera& rasterizer, double error_threshold, geo::Mesh& mesh);

}

#endif

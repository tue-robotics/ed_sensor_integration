#ifndef _world_model_renderer_H_
#define _world_model_renderer_H_

#include <ed/types.h>
#include <geolib/sensors/DepthCamera.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rgbd/View.h>

namespace ed
{
    class RGBDData;
}

namespace edKinect
{

class WorldModelRenderer
{

public:

    WorldModelRenderer();

    virtual ~WorldModelRenderer();

    void render(const ed::RGBDData& sensor_data,
                const ed::WorldModel& world_model,
                float max_range,
                const rgbd::View& view,
                cv::Mat& img,
                pcl::PointCloud<pcl::PointXYZ>& pc,
                std::vector<const ed::Entity*>& pc_entity_ptrs);


private:

    geo::DepthCamera camera_model_;

};

}

#endif

#ifndef ed_sensor_integration_kinect_polygon_height_alm_h_
#define ed_sensor_integration_kinect_polygon_height_alm_h_

#include "ed_sensor_integration/kinect/almodules/rgbd_al_module.h"

#include <ros/publisher.h>

namespace edKinect
{

class PolygonHeightALM : public edKinect::RGBDALModule
{

public:

    PolygonHeightALM();

    void process(const ed::RGBDData& rgbd_data,
                 ed::PointCloudMaskPtr& not_associated_mask,
                 const ed::WorldModel& world_model,
                 ed::UpdateRequest& req);

    void configure(tue::Configuration config);

protected:

    /// Tunable params
    double tolerance_;
    int min_cluster_size_;
    float max_range_;

};

}

#endif

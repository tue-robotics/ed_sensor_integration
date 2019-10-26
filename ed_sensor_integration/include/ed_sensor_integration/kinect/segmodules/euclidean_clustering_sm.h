#ifndef ed_sensor_integration_kinect_euclidean_clustering_sm_h_
#define ed_sensor_integration_kinect_euclidean_clustering_sm_h_

#include "ed_sensor_integration/kinect/segmodules/rgbd_seg_module.h"

namespace edKinect
{

class EuclideanClusteringSM : public edKinect::RGBDSegModule
{

public:

    EuclideanClusteringSM();

    void process(const ed::RGBDData& rgbd_data, std::vector<ed::PointCloudMaskPtr>& segments);

    void configure(tue::Configuration config);

private:
    double tolerance_;
    int min_cluster_size_;

};

}

#endif

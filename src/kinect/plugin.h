#ifndef ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT_PLUGIN_H_

#include "ed_sensor_integration/kinect/almodules/point_normal_alm.h"
#include "ed_sensor_integration/kinect/almodules/polygon_height_alm.h"
#include "ed_sensor_integration/kinect/segmodules/euclidean_clustering_sm.h"

#include <ed/plugin.h>
#include <ed/types.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>


// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

    // Hard coded module declarations
    edKinect::PointNormalALM point_normal_alm_;
    edKinect::PolygonHeightALM polygon_height_alm_;
    edKinect::EuclideanClusteringSM euclidean_clustering_sm_;

    // Tunable parameters from ALmodules
    float voxel_size_;
    float max_range_;
    float clearing_padding_fraction_;
    int normal_k_search_;

};

#endif

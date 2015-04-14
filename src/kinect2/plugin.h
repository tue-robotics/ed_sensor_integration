#ifndef ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_

#include <queue>

#include <ed/plugin.h>
#include <ed/types.h>
#include <ed/helpers/image_publisher.h>
#include <ed/uuid.h>
#include <ed/convex_hull_2d.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

#include "ed_sensor_integration/properties/convex_hull.h"

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    struct Entity
    {
        ed::UUID id;
        geo::Pose3D pose;
        ConvexHull chull;
        std::vector<ed::MeasurementPtr> measurements;
        int count;
    };
    typedef boost::shared_ptr<Entity> EntityPtr;

    std::vector<EntityPtr> local_entities_;

    // PROPERTIES

    ed::PropertyKey<ConvexHull> k_convex_hull_;
    ed::PropertyKey<geo::Pose3D> k_pose_;
    ed::PropertyKey<int> k_clearing_counter_;

    // PARAMETERS

    float association_correspondence_distance_;
    float max_range_;
    float xy_padding_;
    float z_padding_;
    float border_padding_;
    bool debug_;
    int assoc_clear_thr_;
    float clearing_padding_fraction_;

    // VISUALIZATION

    ed::ImagePublisher viz_sensor_normals_;
    ed::ImagePublisher viz_model_normals_;
    ed::ImagePublisher viz_clusters_;
    ed::ImagePublisher viz_update_req_;
    ed::ImagePublisher viz_model_render_;
    ed::ImagePublisher viz_model_sensor_diff_;

    // COMMUNICATION

    std::string topic_;

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;
};

#endif

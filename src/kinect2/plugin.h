#ifndef ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_KINECT2_PLUGIN_H_

#include <queue>

#include <ed/plugin.h>
#include <ed/types.h>
#include <ed/helpers/image_publisher.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

#include "ed/convex_hull.h"

// Locking
#include "ed_sensor_integration/LockEntities.h"
#include "ed_sensor_integration/MeshEntityInView.h"

// ----------------------------------------------------------------------------------------------------

class KinectPlugin : public ed::Plugin
{

public:

    KinectPlugin();

    ~KinectPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    // PARAMETERS

    float association_correspondence_distance_;
    float max_range_;
    float xy_padding_;
    float z_padding_;
    float border_padding_;
    bool debug_;

    // VISUALIZATION

    ed::ImagePublisher viz_sensor_normals_;
    ed::ImagePublisher viz_model_normals_;
    ed::ImagePublisher viz_clusters_;
    ed::ImagePublisher viz_update_req_;
    ed::ImagePublisher viz_model_render_;

    // COMMUNICATION

    std::string topic_;

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    // Locking

    ros::CallbackQueue cb_queue_;

    std::set<ed::UUID> locked_entities_;

    ros::ServiceServer srv_lock_entities_;

    bool srvLockEntities(ed_sensor_integration::LockEntities::Request& req, ed_sensor_integration::LockEntities::Response& res);

    // Meshing

    ed_sensor_integration::MeshEntityInView::Request mesh_request_;

    ros::ServiceServer srv_mesh_entity_in_view_;

    bool srvMeshEntityInView(ed_sensor_integration::MeshEntityInView::Request& req, ed_sensor_integration::MeshEntityInView::Response& res);

};

#endif

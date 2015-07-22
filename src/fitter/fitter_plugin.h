#ifndef ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

// Image capture
#include <rgbd/Client.h>
#include <tf/transform_listener.h>
#include <queue>

// Fitting
#include "beam_model.h"
#include "snapshot.h"

// Model loading
#include <ed/models/model_loader.h>

// Communication
#include "ed_sensor_integration/FitModel.h"
#include "ed_sensor_integration/GetModels.h"
#include "ed_sensor_integration/GetSnapshots.h"
#include "ed_sensor_integration/GetPOIs.h"
#include "ed_sensor_integration/MakeSnapshot.h"
#include "ed_sensor_integration/NavigateTo.h"
#include "ed_sensor_integration/GUIAction.h"
#include <std_srvs/Empty.h>

// Visualization
#include <ed/helpers/image_publisher.h>

#include "gui.h"   // TODO: Navigator is in here!
#include "map_filter.h"

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

// ----------------------------------------------------------------------------------------------------

struct EntityRepresentation2D
{
    unsigned int shape_revision;
    Shape2D shape_2d;
    cv::Mat model_image;
};

// ----------------------------------------------------------------------------------------------------

class FitterPlugin : public ed::Plugin
{

public:

    FitterPlugin();

    ~FitterPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    // Image capture

    rgbd::Client rgbd_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    bool NextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

//    Snapshot last_image_;

    ros::Time last_image_update_;


    // Making snapshots

    Snapshot current_snapshot_;


    // Points of interest

    double min_poi_distance_;

    std::vector<geo::Vec2> pois_;


    // Map fitering

    double wall_height_;

    ros::Time last_wall_creation_;

    MapFilter map_filter_;


    // Fitting

    BeamModel beam_model_;

    void CalculateRanges(const rgbd::Image& image, const geo::Pose3D& sensor_pose_zrp, std::vector<double>& ranges) const;

    void RenderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
                      std::vector<double>& model_ranges, std::vector<int>& identifiers);

    bool FitEntity(const ed::UUID& id, int expected_center_beam, int beam_window, const Shape2D& shape2d,
                   const std::vector<double>& sensor_ranges, const geo::Pose3D& sensor_pose_xya,
                   geo::Pose3D& expected_pose, bool snap_to_expected_beam = true);


    // 2D Entity shapes

    std::map<ed::UUID, EntityRepresentation2D> entity_shapes_;

    const EntityRepresentation2D* GetOrCreateEntity2D(const ed::EntityConstPtr& e);


    // Models

    std::map<std::string, EntityRepresentation2D> models_;

    ed::models::ModelLoader model_loader_;


    // Snapshots

    std::map<ed::UUID, Snapshot> snapshots_;

    unsigned int revision_;


    // Snapshot visualization

    std::set<ed::UUID> changed_entity_ids_;

    void updateSnapshots();

    std::set<ed::UUID> fitted_entity_ids_;

    std::vector<ed::UUID> fitted_entity_ids_stack_;   // Can be used to undo fitting

    std::string snapshot_id_to_first_update_;


    // Navigation

    Navigator navigator_;


    // Visualization

    ros::Publisher pub_gui_actions_;


    // Debug visualization

    ed::ImagePublisher debug_viz_;


    // Communication

    ros::CallbackQueue cb_queue_;

    const ed::WorldModel* world_model_;

    ed::UpdateRequest* update_request_;

    ros::ServiceServer srv_fit_model_;

    bool srvFitModel(ed_sensor_integration::FitModel::Request& req, ed_sensor_integration::FitModel::Response& res);

    ros::ServiceServer srv_get_models_;

    bool srvGetModels(ed_sensor_integration::GetModels::Request& req, ed_sensor_integration::GetModels::Response& res);

    ros::ServiceServer srv_get_snapshots_;

    bool srvGetSnapshots(ed_sensor_integration::GetSnapshots::Request& req, ed_sensor_integration::GetSnapshots::Response& res);

    ros::ServiceServer srv_make_snapshot_;

    bool srvMakeSnapshot(ed_sensor_integration::MakeSnapshot::Request& req, ed_sensor_integration::MakeSnapshot::Response& res);

    ros::ServiceServer srv_get_pois_;

    bool srvGetPOIs(ed_sensor_integration::GetPOIs::Request& req, ed_sensor_integration::GetPOIs::Response& res);

    ros::ServiceServer srv_navigate_to_;

    bool srvNavigateTo(ed_sensor_integration::NavigateTo::Request& req, ed_sensor_integration::NavigateTo::Response& res);

    ros::ServiceServer srv_create_walls_;

    bool srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // Visualization

    void DrawWorldVisualization(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, cv::Mat& canvas);

    void DrawRanges(const std::vector<double>& ranges, const cv::Scalar& color, cv::Mat& canvas);

};

#endif

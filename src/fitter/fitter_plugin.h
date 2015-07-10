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

// Communication
#include "ed_sensor_integration/FitModel.h"
#include "ed_sensor_integration/GetModels.h"
#include "ed_sensor_integration/GetSnapshots.h"

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

// ----------------------------------------------------------------------------------------------------

struct Entity2DModel
{
    unsigned int shape_revision;
    Shape2D shape_2d;
};

// ----------------------------------------------------------------------------------------------------

struct Snapshot
{
    Snapshot() : revision(0) {}

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    unsigned int revision;
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


    // Fitting

    BeamModel beam_model_;


    // 2D Models

    std::map<ed::UUID, Entity2DModel> models_;


    // Snapshots

    std::map<ed::UUID, Snapshot> snapshots_;

    unsigned int revision_;


    // Communication

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_fit_model_;

    bool srvFitModel(ed_sensor_integration::FitModel::Request& req, ed_sensor_integration::FitModel::Response& res);

    ros::ServiceServer srv_get_models_;

    bool srvGetModels(ed_sensor_integration::GetModels::Request& req, ed_sensor_integration::GetModels::Response& res);

    ros::ServiceServer srv_get_snapshots_;

    bool srvGetSnapshots(ed_sensor_integration::GetSnapshots::Request& req, ed_sensor_integration::GetSnapshots::Response& res);

};

#endif

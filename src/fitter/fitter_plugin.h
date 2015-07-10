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

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

// ----------------------------------------------------------------------------------------------------

struct Entity2DModel
{
    unsigned int shape_revision;
    Shape2D shape_2d;
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

};

#endif

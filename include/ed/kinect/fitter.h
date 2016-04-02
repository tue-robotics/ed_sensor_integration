#ifndef ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

#include <rgbd/types.h>

#include "beam_model.h"

// Model loading
#include <ed/models/model_loader.h>

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

// ----------------------------------------------------------------------------------------------------

struct EntityRepresentation2D
{
    unsigned int shape_revision;
    Shape2D shape_2d;
};

// ----------------------------------------------------------------------------------------------------

struct FitterData
{
    std::vector<double> sensor_ranges;
    geo::Pose3D sensor_pose;
    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
};

// ----------------------------------------------------------------------------------------------------

class Fitter
{

public:

    Fitter();

    ~Fitter();

    void processSensorData(const rgbd::Image& image, const geo::Pose3D& sensor_pose, FitterData& data) const;

    void renderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
                      std::vector<double>& model_ranges, std::vector<int>& identifiers);

    bool estimateEntityPose(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change = M_PI);

    EntityRepresentation2D GetOrCreateEntity2D(const ed::EntityConstPtr& e);

private:

    // Fitting

    BeamModel beam_model_;


    // 2D Entity shapes

    std::map<ed::UUID, EntityRepresentation2D> entity_shapes_;


    // Models

    std::map<std::string, EntityRepresentation2D> models_;

    ed::models::ModelLoader model_loader_;

};

#endif

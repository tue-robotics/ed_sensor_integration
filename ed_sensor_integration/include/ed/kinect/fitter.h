#ifndef ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_

#include <exception>

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

#include <rgbd/types.h>

#include "beam_model.h"

// Model loading
#include <ed/models/model_loader.h>

#include <map>
#include <vector>

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

class Candidate;

// ----------------------------------------------------------------------------------------------------

class FitterError: public std::exception
{
public:
    FitterError(const std::string& msg){error_message_ = msg;}

    virtual const char* what() const throw ()
    {
           return error_message_.c_str();
    }
private:
    std::string error_message_;
};

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

struct EstimationInputData
{
    ed::EntityConstPtr entity;
    Shape2D shape2d_transformed;
    geo::Vec2 shape_center;
    int expected_center_beam;
    std::vector<double> sensor_ranges;
};

// ----------------------------------------------------------------------------------------------------

class Fitter
{

public:

    Fitter(uint nr_data_points = 200);  // TODO: remove hard-coded values

    ~Fitter();

    void processSensorData(const rgbd::Image& image, const geo::Pose3D& sensor_pose, FitterData& data) const;

    // ToDo: make private?
    void renderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
                      std::vector<double>& model_ranges, std::vector<int>& identifiers) const;

    bool estimateEntityPose(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change = M_PI) const;

    // ToDo: make private?
    EntityRepresentation2D GetOrCreateEntity2D(const ed::EntityConstPtr& e) const;

private:

    bool estimateEntityPoseImp(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change) const;

    EstimationInputData preProcessInputData(const ed::WorldModel &world, const ed::UUID &id, const geo::Pose3D &expected_pose, const FitterData &data) const; // ToDo: unique_ptr?

    Shape2D get2DShape(ed::EntityConstPtr entity_ptr) const;

    void renderWorldModel2D(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, const ed::UUID& skip_id,
                            std::vector<double>& model_ranges, std::vector<int>& identifiers) const;

    void checkExpectedBeamThroughEntity(const std::vector<double> &model_ranges, ed::EntityConstPtr entity,
                                        const geo::Pose3D &sensor_pose_xya, const int expected_center_beam) const;

    bool evaluateCandidate(const EstimationInputData& static_data, Candidate& candidate) const;

    // Fitting
    BeamModel beam_model_;

    // 2D Entity shapes
    mutable std::map<ed::UUID, EntityRepresentation2D> entity_shapes_;

    // Models
    ed::models::ModelLoader model_loader_;

    uint nr_data_points_;

};

#endif

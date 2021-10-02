#ifndef ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_
#define ED_SENSOR_INTEGRATION_FITTER_PLUGIN_H_

#include <exception>

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

#include <rgbd/types.h>
#include <image_geometry/pinhole_camera_model.h>

#include "beam_model.h"

// Model loading
#include <ed/models/model_loader.h>

#include <map>
#include <vector>

typedef std::vector<std::vector<geo::Vec2> > Shape2D;

/**
 * Forward declaration classes
 */
class Candidate;
class OptimalFit;
struct YawRange;

// ----------------------------------------------------------------------------------------------------

/**
 * @brief The FitterError class exception is thrown in case of errors in the fitter algorithm
 */
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

/**
 * @brief The EntityRepresentation2D struct contains the (downprojected) shape that is used for the
 * fitting
 */
struct EntityRepresentation2D
{
    unsigned int shape_revision;
    Shape2D shape_2d;
};

// ----------------------------------------------------------------------------------------------------

/**
 * @brief The FitterData struct contains the downprojected (hence 2D) sensor readings as well as the
 * sensor pose while taking the image (including decomposition in x, y yaw and roll, pitch, z
 * coordinates)
 */
struct FitterData
{
    std::vector<double> sensor_ranges;
    geo::Pose3D sensor_pose;
    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
};

// ----------------------------------------------------------------------------------------------------

/**
 * @brief The EstimationInputData struct contains processed data from the world model (entity, 2D shape
 * w.r.t. shape center at (0, 0), the shape center in the world model, the beam index that is expected
 * to pass close to the shape center, the rendered 'model ranges' *without the entity to fit*
 *  and the (2D) sensor ranges.
 */
struct EstimationInputData
{
    ed::EntityConstPtr entity;
    Shape2D shape2d_transformed;
    geo::Vec2 shape_center;
    int expected_center_beam;
    std::vector<double> model_ranges;
    std::vector<double> sensor_ranges;
};

// ----------------------------------------------------------------------------------------------------

/**
 * @brief The Fitter class contains the algorithm to do the 2D fit
 */
class Fitter
{

public:

    /**
     * @brief Fitter constructor
     * @param nr_data_points nr_data_points for the beam model
     */
    Fitter(uint nr_data_points = 200);  // TODO: remove hard-coded values

    ~Fitter();

    bool isConfigured() {return configured_;};

    /**
     * @brief processSensorData pre-processes sensor data, i.e., performs a downprojection of the input
     * depth image based on the provided sensor pose and stores the result in the FitterData struct
     * @param image input (depth) image
     * @param sensor_pose pose of the sensor in the world while taking the image
     * @param data processed data is stored here
     */
    void processSensorData(const rgbd::Image& image, const geo::Pose3D& sensor_pose, FitterData& data) const;

    /**
     * @brief estimateEntityPose performs the entity pose estimation. Basically, tries to call estimateEntityPoseImp and
     * return the results, catches any FitterErrors and returns false
     * @param data pre-processed sensor data
     * @param world world model
     * @param id id of the entity we're trying to fit
     * @param expected_pose pose where the entity is expected
     * @param fitted_pose the fitted pose is stored here
     * @param max_yaw_change maximum allowed yaw rotation
     * @return success or failure
     */
    bool estimateEntityPose(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change = M_PI) const;

    /**
     * @brief findOptimum finds the 'optimal' fit by evaluating candidates over all beams and over the entire
     * yaw range.
     * @param input_data EstimationInputData: everything that's required
     * @param yaw_range min and max yaw range to sample over
     * @return pointer to the optimum
     */
    std::unique_ptr<OptimalFit> findOptimum(const EstimationInputData& input_data, const YawRange& yaw_range) const;

    /**
     * @brief GetOrCreateEntity2D returns the downprojected shape of the entity. If it's already in the cache,
     * it's returned directly. If not, it's obtained from the entity
     * @param e input entity
     * @return 2D entity representation
     */
    EntityRepresentation2D GetOrCreateEntity2D(const ed::EntityConstPtr& e) const;

    /**
     * @brief configure the beam model (nr of data points and focal length) according to the camera you are using.
     * @param caminfo camera info
     * @return
     */
    void configureBeamModel(const image_geometry::PinholeCameraModel& caminfo);

private:

    /**
     * @brief renderEntity renders the entity in 2D using the beam model with the provided (2D) sensor pose.
     *  The ranges of the simulated measurement are stored in 'model ranges' and the 'identifier' is stored
     * in the 'identifiers' vector in order to be able to distinguish multiple entity shapes.
     * @param e entity to render
     * @param sensor_pose_xya 2D sensor pose (i.e., only x, y and yaw are used)
     * @param identifier
     * @param model_ranges
     * @param identifiers
     */
    void renderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
                      std::vector<double>& model_ranges, std::vector<int>& identifiers) const;

    /**
     * @brief estimateEntityPoseImp actual implementation of the entity pose estimation. Preprocess input data
     * iterates over all beams and the yaw range to compuate the best fit. Finally, a correction of the sensor
     * pose is computed and the result is transformed back to 3D
     * @param data pre-processed sensor data
     * @param world world model
     * @param id id of the entity we're trying to fit
     * @param expected_pose pose where the entity is expected
     * @param fitted_pose the fitted pose is stored here
     * @param max_yaw_change maximum allowed yaw rotation
     * @return success or failure
     */
    bool estimateEntityPoseImp(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change) const;

    /**
     * @brief preProcessInputData pre-processes the inputs for the fitting algorithm, i.e., gets the shape of
     * the entity, compute its center, transform the shape to (0, 0), and compute the expected center beam.
     * Everything is stored in a struct that can be used (const) throughout the rest of the algorithm
     * @param world world model
     * @param id id of the entity to fit
     * @param expected_pose expected pose of the entity
     * @param data input sensor data
     * @return EstimationInputData used throughout the rest of the algorithm
     */
    EstimationInputData preProcessInputData(const ed::WorldModel &world, const ed::UUID &id, const geo::Pose3D &expected_pose, const FitterData &data) const;

    /**
     * @brief get2DShape gets the downprojected, 2D shape from an entity
     * @param entity_ptr points to the entity
     * @return the computed shape
     */
    Shape2D get2DShape(ed::EntityConstPtr entity_ptr) const;

    /**
     * @brief renderWorldModel2D renders all world model entities
     * @param world world model
     * @param sensor_pose_xya sensor pose
     * @param skip_id id to skip (so we don't render the entity we're fitting
     * @param model_ranges data is stored here
     * @param identifiers to distinguish which 'range' belongs to which entity. In this case, -1 will
     * be used as an identifier
     */
    void renderWorldModel2D(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, const ed::UUID& skip_id,
                            std::vector<double>& model_ranges, std::vector<int>& identifiers) const;

    /**
     * @brief checkExpectedBeamThroughEntity checks if the expected center beam passes through the entity. If
     * not: something went wrong
     * @param model_ranges
     * @param entity
     * @param sensor_pose_xya
     * @param expected_center_beam
     */
    void checkExpectedBeamThroughEntity(const std::vector<double> &model_ranges, ed::EntityConstPtr entity,
                                        const geo::Pose3D &sensor_pose_xya, const int expected_center_beam) const;

    /**
     * @brief evaluateCandidate renders the model, transform the model along the beam direction to get this
     * distance right and render once again
     * @param static_data 'static' input data that was the result from the pre-processing
     * @param candidate contains the 'candidate' beam index and yaw that is evaluated
     * @return
     */
    bool evaluateCandidate(const EstimationInputData& static_data, Candidate& candidate) const;

    // Fitting
    BeamModel beam_model_;

    // 2D Entity shapes
    mutable std::map<ed::UUID, EntityRepresentation2D> entity_shapes_;

    // Models
    ed::models::ModelLoader model_loader_;

    uint nr_data_points_;
    bool configured_;

};

#endif

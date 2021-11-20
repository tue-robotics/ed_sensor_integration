// Generic
#include <limits>

// Logging
#include <ros/console.h>

#include "ed/kinect/fitter.h"
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/logging.h>
#include <geolib/Shape.h>

// Image capture
#include <rgbd/image.h>
#include <geolib/ros/tf_conversions.h>
#include <rgbd/view.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

// 2D model creation
#include "ed/kinect/mesh_tools.h"

// Communication
#include <ed_sensor_integration_msgs/ImageBinary.h>


const double ERROR_THRESHOLD = 1e5;


struct YawRange
{
    double min, max;
};

// ----------------------------------------------------------------------------------------------------

YawRange computeYawRange(const geo::Pose3D& sensor_pose_xya, const geo::Pose3D& expected_entity_pose, const double& max_yaw_change)
{
    geo::Pose3D expected_pose_SENSOR = sensor_pose_xya.inverse() * expected_entity_pose;
    double expected_yaw_SENSOR;
    {
        tf::Matrix3x3 m;
        geo::convert(expected_pose_SENSOR.R, m);
        double roll, pitch;
        m.getRPY(roll, pitch, expected_yaw_SENSOR);
    }

    double min_yaw = expected_yaw_SENSOR - max_yaw_change;
    double max_yaw = expected_yaw_SENSOR + max_yaw_change;
    return {min_yaw, max_yaw};
}

// ----------------------------------------------------------------------------------------------------

geo::Vec2 computeShapeCenter(const Shape2D& shape2d)
{
    // ToDo: make member method? This doesn't make any sense as a separate method
    // since you need to know the internals of a Shape2D
    geo::Vec2 shape_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    geo::Vec2 shape_max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min());

    for(uint i = 0; i < shape2d.size(); ++i)
    {
        const std::vector<geo::Vec2>& contour = shape2d[i];
        for(uint j = 0; j < contour.size(); ++j)
        {
            const geo::Vec2& p = contour[j];
            shape_min.x = std::min(shape_min.x, p.x);
            shape_min.y = std::min(shape_min.y, p.y);
            shape_max.x = std::max(shape_max.x, p.x);
            shape_max.y = std::max(shape_max.y, p.y);
        }
    }

    return 0.5 * (shape_min + shape_max);
}

// ----------------------------------------------------------------------------------------------------

Shape2D transformShape2D(const Shape2D& original_shape, const geo::Vec2& transformation)
{
    Shape2D shape2d_transformed = original_shape;
    for(uint i = 0; i < shape2d_transformed.size(); ++i)
    {
        std::vector<geo::Vec2>& contour_transformed = shape2d_transformed[i];
        for(uint j = 0; j < contour_transformed.size(); ++j)
            contour_transformed[j] -= transformation;
    }
    return shape2d_transformed;
}

// ----------------------------------------------------------------------------------------------------

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

// Convert a 3D transform with only a x, y and yaw component to a 2D transform
geo::Transform2 XYYawToTransform2(const geo::Pose3D& pose)
{
    return geo::Transform2(geo::Mat2(pose.R.xx, pose.R.xy, pose.R.yx, pose.R.yy), geo::Vec2(pose.t.x, pose.t.y));
}

// ----------------------------------------------------------------------------------------------------

class OptimalFit
{
public:

    OptimalFit() : error(10.0 * ERROR_THRESHOLD) {}

    ~OptimalFit(){}

    void update(const geo::Transform2& new_pose, const double& new_error)
    {
        pose = new_pose;
        error = new_error;
    }

    double getError(){return error;}
    geo::Transform2 getPose(){return pose;}

private:
    geo::Transform2 pose;
    double error;
};

// ----------------------------------------------------------------------------------------------------

class Candidate
{
public:

    Candidate(std::shared_ptr<BeamModel> beam_model_ptr) : beam_model_ptr_(beam_model_ptr)
    {
        test_ranges.resize(beam_model_ptr_->num_beams(), 0.0);
    }
    void initialize(const uint i_beam, const double yaw)
    {
        beam_index = i_beam;
        beam_length = beam_model_ptr_->rays()[i_beam].length();
        beam_direction = beam_model_ptr_->rays()[i_beam] / beam_length;
        pose.setRotation(yaw);
        pose.setOrigin(beam_direction * 10);  // JL: why this factor 10?
        std::fill(test_ranges.begin(), test_ranges.end(), 0.0);
    }

    ~Candidate(){}

    // ToDo: how can we make this private? Use smart pointers as well?
    // It's not desirable to make the beam model aware of this datastructure
    uint beam_index;
    double beam_length;
    geo::Vec2 beam_direction;
    geo::Transform2 pose;
    std::vector<double> test_ranges;

private:
    std::shared_ptr<BeamModel> beam_model_ptr_;

};

// ----------------------------------------------------------------------------------------------------

double computeFittingError(const std::vector<double>& test_ranges, const std::vector<double>& sensor_ranges)
{
    int n = 0;
    double total_error = 0;
    for(unsigned int i = 0; i < test_ranges.size(); ++i)
    {
        double ds = sensor_ranges[i];
        double dm = test_ranges[i];

        if (ds <= 0)
            continue;

        ++n;

        if (dm <= 0)
        {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs(ds - dm);
        if (diff < 0.1)
            total_error += diff;
        else
        {
            if (ds > dm)
                total_error += 1;
            else
                total_error += 0.1;
        }
    }

    double error = total_error / n;
    return error;
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D computeFittedPose(const geo::Transform2& pose_sensor, ed::EntityConstPtr entity, const geo::Pose3D& sensor_pose_xya)
{
    geo::Pose3D pose_3d;
    pose_3d.t = geo::Vec3(pose_sensor.t.x, pose_sensor.t.y, entity->pose().t.z);
    pose_3d.R = geo::Mat3::identity();
    pose_3d.R.xx = pose_sensor.R.xx;
    pose_3d.R.xy = pose_sensor.R.xy;
    pose_3d.R.yx = pose_sensor.R.yx;
    pose_3d.R.yy = pose_sensor.R.yy;

    return sensor_pose_xya * pose_3d;
}

// ----------------------------------------------------------------------------------------------------

Fitter::Fitter(uint nr_data_points) :
    nr_data_points_(nr_data_points)
{
    beam_model_.initialize(4, nr_data_points);
    configured_ = false;
}

// ----------------------------------------------------------------------------------------------------

Fitter::~Fitter()
{
}

// ----------------------------------------------------------------------------------------------------

bool Fitter::estimateEntityPose(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                                const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change) const
{
    try
    {
        return estimateEntityPoseImp(data, world, id, expected_pose, fitted_pose, max_yaw_change);
    }
    catch (const FitterError& error)
    {
        ROS_ERROR_STREAM("Fitter error: " << error.what());
        return false;
    }
}

// ----------------------------------------------------------------------------------------------------

bool Fitter::estimateEntityPoseImp(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                                   const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change) const
{
    const EstimationInputData estimation_input_data = preProcessInputData(world, id, expected_pose, data);

    // -------------------------------------
    // Compute yaw range
    YawRange yaw_range = computeYawRange(data.sensor_pose_xya, expected_pose, max_yaw_change);

    // -------------------------------------
    // Fit
    std::unique_ptr<OptimalFit> current_optimum = findOptimum(estimation_input_data, yaw_range);

    double error_threshold = ERROR_THRESHOLD;
    if (current_optimum->getError() > error_threshold)
    {
        throw FitterError("Error of best fit exceeds threshold");
    }

    // Correct for shape transformation
    geo::Transform2 best_pose_SENSOR = current_optimum->getPose();
    best_pose_SENSOR.t += best_pose_SENSOR.R * -estimation_input_data.shape_center;

    // Convert to 3D Pose
    fitted_pose = computeFittedPose(best_pose_SENSOR, estimation_input_data.entity, data.sensor_pose_xya);

    return true;
}

// ----------------------------------------------------------------------------------------------------

EstimationInputData Fitter::preProcessInputData(const ed::WorldModel& world, const ed::UUID& id, const geo::Pose3D& expected_pose, const FitterData& data) const
{
    EstimationInputData result;

    // Get entity for which to fit the pose
    result.entity = world.getEntity(id);

    // -------------------------------------
    // Get 2D contour
    const Shape2D& shape2d = get2DShape(result.entity);

    // -------------------------------------
    // Determine center of the shape
    result.shape_center = computeShapeCenter(shape2d);

    // -------------------------------------
    // Transform shape2d such that origin is in the center
    result.shape2d_transformed = transformShape2D(shape2d, result.shape_center);

    // -------------------------------------
    // Render world model objects
    result.model_ranges.resize(nr_data_points_, 0);
    std::vector<int> dummy_identifiers(nr_data_points_, -1);
    renderWorldModel2D(world, data.sensor_pose_xya, id, result.model_ranges, dummy_identifiers);

    // -------------------------------------
    // Calculate the beam which shoots through the expected position of the entity
    geo::Vec3 expected_pos_sensor = data.sensor_pose_xya.inverse() * expected_pose.t;
    result.expected_center_beam = beam_model_.CalculateBeam(expected_pos_sensor.x, expected_pos_sensor.y);

    // ----------------------------------------------------
    // Check that we can see the shape in its expected pose
    checkExpectedBeamThroughEntity(result.model_ranges, result.entity, data.sensor_pose_xya, result.expected_center_beam);

    result.sensor_ranges = data.sensor_ranges;
    return result;
}

// ----------------------------------------------------------------------------------------------------

std::unique_ptr<OptimalFit> Fitter::findOptimum(const EstimationInputData& input_data, const YawRange& yaw_range) const
{
    std::unique_ptr<OptimalFit> current_optimum(new OptimalFit);
    std::shared_ptr<BeamModel> beam_model_ptr = std::make_shared<BeamModel>(beam_model_);
    Candidate candidate(beam_model_ptr);
    bool valid_optimum = false;

    for(uint i_beam = 0; i_beam < nr_data_points_; ++i_beam)
    {
        // Iterate over the yaw range
        for(double yaw = yaw_range.min; yaw < yaw_range.max; yaw += 0.1)
        {
            // Initialize candidate solution
            candidate.initialize(i_beam, yaw);

            // And render it
            if (!evaluateCandidate(input_data, candidate))
                continue;

            // Calculate error
            double error = computeFittingError(candidate.test_ranges, input_data.sensor_ranges);

            // Update optimum
            if (error < current_optimum->getError())
            {
                current_optimum->update(candidate.pose, error);
                // reject an optimum value found at the boundary of the search space as it is not a global maximum.
                valid_optimum = !(i_beam==0 || i_beam==nr_data_points_-1);
            }
        }
    }
    if (valid_optimum)
        return current_optimum;
    std::unique_ptr<OptimalFit> invalid_optimum(new OptimalFit);
    return invalid_optimum;
}

// ----------------------------------------------------------------------------------------------------

bool Fitter::evaluateCandidate(const EstimationInputData &static_data, Candidate& candidate) const
{
    // Render the entity model *exluding* other entities in the world model
    std::vector<int> dummy_identifiers(nr_data_points_, -1); // ToDo: prevent redeclaration?
    beam_model_.RenderModel(static_data.shape2d_transformed, candidate.pose, 0, candidate.test_ranges, dummy_identifiers);

    double ds = static_data.sensor_ranges[candidate.beam_index]; // Transformed sensor reading (distance measured along beam)
    double dm = candidate.test_ranges[candidate.beam_index]; // Distance along the beam to the candidate entity

    if (ds <= 0 || dm <= 0)
        return false;

	// Correct the entity pose in the direction of the beam
    candidate.pose.t += candidate.beam_direction * ((ds - dm) * candidate.beam_length); // JL: Why multiply with beam_length (or, at least, 'l')?

    // Render model on top of the rendered data of the other entities in the world model
    candidate.test_ranges = static_data.model_ranges;
    std::vector<int> identifiers(nr_data_points_, 0);  // ToDo: prevent redeclaration?
    beam_model_.RenderModel(static_data.shape2d_transformed, candidate.pose, 1, candidate.test_ranges, identifiers);

    if (identifiers[static_data.expected_center_beam] != 1)  // expected center beam MUST contain the rendered model
        return false;

    return true;

}

// ----------------------------------------------------------------------------------------------------

Shape2D Fitter::get2DShape(ed::EntityConstPtr entity_ptr) const
{
    if (!entity_ptr->shape())
        throw FitterError("Entity " + entity_ptr->id().str() + " has no shape");

    EntityRepresentation2D repr_2d = GetOrCreateEntity2D(entity_ptr);
    if (repr_2d.shape_2d.empty())
        throw FitterError("No conversion to 2D shape for entity " + entity_ptr->id().str());

    return repr_2d.shape_2d;
}

// ----------------------------------------------------------------------------------------------------

void Fitter::renderWorldModel2D(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, const ed::UUID& skip_id,
                                std::vector<double>& model_ranges, std::vector<int>& identifiers) const
{
    // ToDo: re-implement: use WM copy and remove robots, furniture object. Render that
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->id() == skip_id) // Skip entity id that needs to be fitted
            continue;

        if (e->hasFlag("self")) // Skip the robot itself
            continue;

        std::string id_str = e->id().str();
        if (id_str.size() >= 6 && id_str.substr(0, 6) == "sergio")
            continue;

        if (id_str.size() >= 5 && id_str.substr(0, 5) == "amigo")
            continue;

        renderEntity(e, sensor_pose_xya, -1, model_ranges, identifiers);
    }

}

// ----------------------------------------------------------------------------------------------------

void Fitter::checkExpectedBeamThroughEntity(const std::vector<double>& model_ranges,
                                            ed::EntityConstPtr entity,
                                            const geo::Pose3D& sensor_pose_xya,
                                            const int expected_center_beam) const
{
    std::vector<double> expected_ranges(nr_data_points_, 0);
    expected_ranges = model_ranges;
    std::vector<int> expected_identifiers(nr_data_points_, 0);
    renderEntity(entity, sensor_pose_xya, 1, expected_ranges, expected_identifiers);

    if (expected_identifiers[expected_center_beam] != 1)  // expected center beam MUST contain the rendered model
        throw FitterError("Expected beam does not go through entity");
}

// ----------------------------------------------------------------------------------------------------

void Fitter::configureBeamModel(const image_geometry::PinholeCameraModel& caminfo)
{
    uint nr_beams = std::min(200, caminfo.fullResolution().width); // don't use more data points than the resolution of your camera
    double fx = caminfo.fx();
    double fx_resize = fx * nr_beams / caminfo.fullResolution().width; // Reducing nr of data points will require a different focal length
    double w = 2 * nr_beams / fx_resize; // reverse calculation of the width of the beam model.
    beam_model_.initialize(w, nr_beams);
    nr_data_points_ = nr_beams;
    ROS_INFO("Configured fitter with %i beams and a focal length of %f", nr_beams, fx);
    configured_ = true;
}

// ----------------------------------------------------------------------------------------------------
void Fitter::processSensorData(const rgbd::Image& image, const geo::Pose3D& sensor_pose, FitterData& data) const
{
    data.sensor_pose = sensor_pose;
    decomposePose(sensor_pose, data.sensor_pose_xya, data.sensor_pose_zrp);

    const cv::Mat& depth = image.getDepthImage();
    rgbd::View view(image, depth.cols);

    std::vector<double>& ranges = data.sensor_ranges;

    if (ranges.size() != beam_model_.num_beams())
        ranges.resize(beam_model_.num_beams(), 0);

    data.fx = beam_model_.fx();

    for(int x = 0; x < depth.cols; ++x)
    {
        for(int y = 0; y < depth.rows; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
                continue;

            geo::Vector3 p_sensor = view.getRasterizer().project2Dto3D(x, y) * d;
            geo::Vector3 p_floor = data.sensor_pose_zrp * p_sensor;

            if (p_floor.z < 0.2) // simple floor filter
                continue;

            int i = beam_model_.CalculateBeam(p_floor.x, p_floor.y);
            if (i >= 0 && i < static_cast<int>(ranges.size()))
            {
                double& r = ranges[i];
                if (r == 0 || p_floor.y < r)
                    r = p_floor.y;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

EntityRepresentation2D Fitter::GetOrCreateEntity2D(const ed::EntityConstPtr& e) const
{
    std::map<ed::UUID, EntityRepresentation2D>::const_iterator it_model = entity_shapes_.find(e->id());
    // ToDo: this does not accomodate for different shape revisions.
    if (it_model != entity_shapes_.end())
        return it_model->second;

    // Decompose entity pose into X Y YAW and Z ROLL PITCH
    geo::Pose3D pose_xya;
    geo::Pose3D pose_zrp;
    decomposePose(e->pose(), pose_xya, pose_zrp);

    EntityRepresentation2D& entity_model = entity_shapes_[e->id()];
    dml::project2D(e->shape()->getMesh().getTransformed(pose_zrp), entity_model.shape_2d);

    return entity_model;
}

// ----------------------------------------------------------------------------------------------------

void Fitter::renderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
                  std::vector<double>& model_ranges, std::vector<int>& identifiers) const
{
    geo::Transform2 sensor_pose_xya_2d = XYYawToTransform2(sensor_pose_xya);

    if (model_ranges.size() != beam_model_.num_beams())
        model_ranges.resize(beam_model_.num_beams(), 0);

    if (!e->shape() || !e->has_pose())
        return;

    // Decompose entity pose into X Y YAW and Z ROLL PITCH
    geo::Pose3D pose_xya;
    geo::Pose3D pose_zrp;
    decomposePose(e->pose(), pose_xya, pose_zrp);

    EntityRepresentation2D e2d = GetOrCreateEntity2D(e);

    geo::Transform2 pose_2d_SENSOR = sensor_pose_xya_2d.inverse() * XYYawToTransform2(pose_xya);

    beam_model_.RenderModel(e2d.shape_2d, pose_2d_SENSOR, identifier, model_ranges, identifiers);
}

// ----------------------------------------------------------------------------------------------------

#include "ed/kinect/fitter.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/logging.h>
#include <geolib/Shape.h>

// Image capture
#include <rgbd/Image.h>
#include <geolib/ros/tf_conversions.h>

//
#include <rgbd/View.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

// 2D model creation
#include "ed/kinect/mesh_tools.h"

// Communication
#include "ed_sensor_integration/ImageBinary.h"

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

Fitter::Fitter()
{
    beam_model_.initialize(4, 200);  // TODO: remove hard-coded values
}

// ----------------------------------------------------------------------------------------------------

Fitter::~Fitter()
{
}

// ----------------------------------------------------------------------------------------------------

bool Fitter::estimateEntityPose(const FitterData& data, const ed::WorldModel& world, const ed::UUID& id,
                                const geo::Pose3D& expected_pose, geo::Pose3D& fitted_pose, double max_yaw_change)
{
    const std::vector<double>& sensor_ranges = data.sensor_ranges;

    // -------------------------------------
    // Get 2D contour

    ed::EntityConstPtr e = world.getEntity(id);
    if (!e->shape())
        return false;

    EntityRepresentation2D repr_2d = GetOrCreateEntity2D(e);
    if (repr_2d.shape_2d.empty())
        return false;

    const Shape2D& shape2d = repr_2d.shape_2d;

    // -------------------------------------
    // Calculate the beam which shoots through the expected position of the entity

    geo::Vec3 expected_pos_SENSOR = data.sensor_pose_xya.inverse() * expected_pose.t;
    int expected_center_beam = beam_model_.CalculateBeam(expected_pos_SENSOR.x, expected_pos_SENSOR.y);

    // -------------------------------------
    // Render world model objects

    std::vector<double> model_ranges(sensor_ranges.size(), 0);
    std::vector<int> dummy_identifiers(sensor_ranges.size(), -1);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->id() == id) // Skip entity id that needs to be fitted
            continue;

        if (e->hasFlag("self")) // Skip the robot itself
            continue;

        std::string id_str = e->id().str();
        if (id_str.size() >= 6 && id_str.substr(0, 6) == "sergio")
            continue;

        if (id_str.size() >= 5 && id_str.substr(0, 5) == "amigo")
            continue;

        renderEntity(e, data.sensor_pose_xya, -1, model_ranges, dummy_identifiers);
    }

    geo::Pose3D expected_pose_SENSOR = data.sensor_pose_xya.inverse() * expected_pose;
    double expected_yaw_SENSOR;
    {
        tf::Matrix3x3 m;
        geo::convert(expected_pose_SENSOR.R, m);
        double roll, pitch;
        m.getRPY(roll, pitch, expected_yaw_SENSOR);
    }

    double min_yaw = expected_yaw_SENSOR - max_yaw_change;
    double max_yaw = expected_yaw_SENSOR + max_yaw_change;

    // -------------------------------------
    // Determine center of the shape

    geo::Vec2 shape_min(1e6, 1e6);
    geo::Vec2 shape_max(-1e6, -1e6);

    for(int i = 0; i < shape2d.size(); ++i)
    {
        const std::vector<geo::Vec2>& contour = shape2d[i];
        for(int j = 0; j < contour.size(); ++j)
        {
            const geo::Vec2& p = contour[j];
            shape_min.x = std::min(shape_min.x, p.x);
            shape_min.y = std::min(shape_min.y, p.y);
            shape_max.x = std::max(shape_max.x, p.x);
            shape_max.y = std::max(shape_max.y, p.y);
        }
    }

    geo::Vec2 shape_center = 0.5 * (shape_min + shape_max);

    // -------------------------------------
    // Transform shape2d such that origin is in the center

    Shape2D shape2d_transformed = shape2d;

    for(int i = 0; i < shape2d_transformed.size(); ++i)
    {
        std::vector<geo::Vec2>& contour_transformed = shape2d_transformed[i];
        for(int j = 0; j < contour_transformed.size(); ++j)
            contour_transformed[j] -= shape_center;
    }

    // -------------------------------------
    // Fit

    double min_error = 1e9;
    geo::Transform2 best_pose_SENSOR;

    for(int i_beam = 0; i_beam < sensor_ranges.size(); ++i_beam)
    {
        double l = beam_model_.rays()[i_beam].length();
        geo::Vec2 r = beam_model_.rays()[i_beam] / l;

        for(double yaw = min_yaw; yaw < max_yaw; yaw += 0.1)
        {
            // ----------------
            // Calculate rotation

            double cos_alpha = cos(yaw);
            double sin_alpha = sin(yaw);
            geo::Mat2 rot(cos_alpha, -sin_alpha, sin_alpha, cos_alpha);
            geo::Transform2 pose(rot, r * 10);

            // ----------------
            // Determine initial pose based on measured range

            std::vector<double> test_ranges(sensor_ranges.size(), 0);
            beam_model_.RenderModel(shape2d_transformed, pose, 0, test_ranges, dummy_identifiers);

            double ds = sensor_ranges[i_beam];
            double dm = test_ranges[i_beam];

            if (ds <= 0 || dm <= 0)
                continue;

            pose.t += r * ((ds - dm) * l);

            // ----------------
            // Render model

            test_ranges = model_ranges;
            std::vector<int> identifiers(sensor_ranges.size(), 0);
            beam_model_.RenderModel(shape2d_transformed, pose, 1, test_ranges, identifiers);

            if (identifiers[expected_center_beam] != 1)  // expected center beam MUST contain the rendered model
                continue;

            // ----------------
            // Calculate error

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

            if (error < min_error)
            {
                best_pose_SENSOR = pose;
                min_error = error;
            }
        }
    }

    if (min_error > 1e5)
    {
//        std::cout << "No pose found!" << std::endl;
        return false;
    }

    // Correct for shape transformation
    best_pose_SENSOR.t += best_pose_SENSOR.R * -shape_center;

//    std::cout << "Found a pose: " << best_pose_SENSOR << std::endl;

    // Convert to 3D Pose

    geo::Pose3D pose_3d;
    pose_3d.t = geo::Vec3(best_pose_SENSOR.t.x, best_pose_SENSOR.t.y, e->pose().t.z);
    pose_3d.R = geo::Mat3::identity();
    pose_3d.R.xx = best_pose_SENSOR.R.xx;
    pose_3d.R.xy = best_pose_SENSOR.R.xy;
    pose_3d.R.yx = best_pose_SENSOR.R.yx;
    pose_3d.R.yy = best_pose_SENSOR.R.yy;

    fitted_pose = data.sensor_pose_xya * pose_3d;

    return true;
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
            if (i >= 0 && i < ranges.size())
            {
                double& r = ranges[i];
                if (r == 0 || p_floor.y < r)
                    r = p_floor.y;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

EntityRepresentation2D Fitter::GetOrCreateEntity2D(const ed::EntityConstPtr& e)
{
    std::map<ed::UUID, EntityRepresentation2D>::const_iterator it_model = entity_shapes_.find(e->id());
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
                  std::vector<double>& model_ranges, std::vector<int>& identifiers)
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

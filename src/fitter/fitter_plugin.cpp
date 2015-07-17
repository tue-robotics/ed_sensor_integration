#include "fitter_plugin.h"

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
#include "mesh_tools.h"

// Communication
#include "ed_sensor_integration/ImageBinary.h"

// Snapshot visualization
#include "gui.h"

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

bool ImageToMsg(const cv::Mat& image, const std::string& encoding, ed_sensor_integration::ImageBinary& msg)
{
    msg.encoding = encoding;

    cv::Mat rgb_image;
    if (image.channels() == 1)
    {
        // depth image
        rgb_image = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        for(unsigned int i = 0; i < rgb_image.rows * rgb_image.cols; ++i)
            rgb_image.at<cv::Vec3b>(i) = (image.at<float>(i) / 10) * cv::Vec3b(255, 255, 255);
    }
    else
    {
        rgb_image = image;
    }

    if (encoding == "jpg")
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", rgb_image, msg.data, rgb_params)) {
            std::cout << "RGB image compression failed" << std::endl;
            return false;
        }
    }
    else if (encoding == "png")
    {
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = CV_IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        if (!cv::imencode(".png", rgb_image, msg.data, params)) {
            std::cout << "PNG image compression failed" << std::endl;
            return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

typedef std::vector<unsigned int> Segment;

void segment(const std::vector<double>& ranges, const BeamModel& beam_model, double segment_depth_threshold,
             double max_gap_size, double min_segment_size, std::vector<Segment>& segments)
{
    // Find first valid value
    Segment current_segment;
    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        if (ranges[i] > 0)
        {
            current_segment.push_back(i);
            break;
        }
    }

    if (current_segment.empty())
        return;

    int gap_size = 0;
    for(unsigned int i = current_segment.front(); i < ranges.size(); ++i)
    {
        float rs = ranges[i];

        if (rs == 0 || std::abs(rs - ranges[current_segment.back()]) > segment_depth_threshold)
        {
            // Found a gap
            ++gap_size;

            if (gap_size >= max_gap_size)
            {
                i = current_segment.back() + 1;

                geo::Vec2 p1 = beam_model.CalculatePoint(current_segment.front(), ranges[current_segment.front()]);
                geo::Vec2 p2 = beam_model.CalculatePoint(current_segment.back(), ranges[current_segment.back()]);
                if ((p2 - p1).length2() > min_segment_size * min_segment_size)
                    segments.push_back(current_segment);

                current_segment.clear();

                // Find next good value
                while(ranges[i] == 0 && i < ranges.size())
                    ++i;

                current_segment.push_back(i);
            }
        }
        else
        {
            gap_size = 0;
            current_segment.push_back(i);
        }
    }

    geo::Vec2 p1 = beam_model.CalculatePoint(current_segment.front(), ranges[current_segment.front()]);
    geo::Vec2 p2 = beam_model.CalculatePoint(current_segment.back(), ranges[current_segment.back()]);
    if ((p2 - p1).length2() > min_segment_size * min_segment_size)
        segments.push_back(current_segment);
}

// ====================================================================================================

// ----------------------------------------------------------------------------------------------------

FitterPlugin::FitterPlugin() : tf_listener_(0), revision_(0), need_snapshot_update_(false)
{
}

// ----------------------------------------------------------------------------------------------------

FitterPlugin::~FitterPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    // Initialize RGBD client
    std::string rgbd_topic;
    if (config.value("topic", rgbd_topic))
        rgbd_client_.intialize(rgbd_topic);

    config.value("min_poi_distance", min_poi_distance_);

    // Load models (used for fitting)
    if (config.readArray("models"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                continue;

            // Load model data
            ed::UpdateRequest req;
            std::stringstream error;
            ed::UUID tmp_id = "id";

            if (!model_loader_.create(tmp_id, name, req, error))
            {
                ed::log::error() << "While loading model '" << name << "': " << error.str() << std::endl;
                continue;
            }

            std::map<ed::UUID, geo::ShapeConstPtr>::const_iterator it_shape = req.shapes.find(tmp_id);
            if (it_shape == req.shapes.end())
            {
                ed::log::error() << "While loading model '" << name << "': model does not have a shape" << std::endl;
                continue;
            }

            geo::ShapeConstPtr shape = it_shape->second;

            EntityRepresentation2D& model = models_[name];
            dml::project2D(shape->getMesh(), model.shape_2d);
        }

        config.endArray();
    }

    beam_model_.initialize(2, 200);

    tf_listener_ = new tf::TransformListener;

    // Initialize services
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_fit_model_ = nh.advertiseService("gui/fit_model", &FitterPlugin::srvFitModel, this);
    srv_get_models_ = nh.advertiseService("gui/get_models", &FitterPlugin::srvGetModels, this);
    srv_get_snapshots_ = nh.advertiseService("gui/get_snapshots", &FitterPlugin::srvGetSnapshots, this);
    srv_make_snapshot_ = nh.advertiseService("make_snapshot", &FitterPlugin::srvMakeSnapshot, this);
    srv_get_pois_ = nh.advertiseService("get_pois", &FitterPlugin::srvGetPOIs, this);
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    world_model_ = &world;
    update_request_ = &req;

    // -------------------------------------
    // Update snapshots if needed

    if (need_snapshot_update_)
        updateSnapshots();

    // -------------------------------------
    // Handle service requests

    make_snapshot_ = false;
    cb_queue_.callAvailable();

    // -------------------------------------
    // Grab image and sensor pose

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;
    if (!NextImage("/map", image, sensor_pose))
        return;

    // -------------------------------------
    // Decompose sensor pose into X Y YAW and Z ROLL PITCH

    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    decomposePose(sensor_pose, sensor_pose_xya, sensor_pose_zrp);
    geo::Transform2 sensor_pose_xya_2d = XYYawToTransform2(sensor_pose_xya);

    // -------------------------------------
    // Calculate virtual rgbd beam ranges

    std::vector<double> ranges;
    CalculateRanges(*image, sensor_pose_zrp, ranges);

    // -------------------------------------
    // Render world model objects (without fitted objects)

    std::vector<double> model_ranges_background(beam_model_.num_beams(), 0);
    std::vector<int> rendered_indices(beam_model_.num_beams(), -1);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (fitted_entity_ids_.find(e->id()) == fitted_entity_ids_.end())
            RenderEntity(e, sensor_pose_xya, -1, model_ranges_background, rendered_indices);
    }

    // -------------------------------------
    // Render fitted objects

    std::vector<ed::EntityConstPtr> fitted_entities;

    std::vector<double> model_ranges = model_ranges_background;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (fitted_entity_ids_.find(e->id()) != fitted_entity_ids_.end())
        {
            int identifier = fitted_entities.size();
            RenderEntity(e, sensor_pose_xya, identifier, model_ranges, rendered_indices);
            fitted_entities.push_back(e);
        }
    }

    // -------------------------------------
    // Determine position errors of fitted objects

    std::vector<int> fitted_entities_num_beams(fitted_entities.size(), 0); // number of beams corresponding to this entity
    std::vector<double> fitted_entities_errors(fitted_entities.size(), 0);    // error corresponding to this entity

    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double rs = ranges[i];
        if (rs == 0)
            continue;

        int i_entity = rendered_indices[i];
        if (i_entity < 0) // Does not belong to fitted entity
            continue;

        ++fitted_entities_num_beams[i_entity];

        double rm = model_ranges[i];
        double diff = rm - rs;

        if (std::abs(diff) < 0.1)
            continue;

        double err = 0;
        if (diff > 0)
        {
            // occlusion
            err = 0.1;
        }
        else
        {
            // rendered point not there...
            err = 1;
        }

        fitted_entities_errors[i_entity] += err;
    }

    for(unsigned int i = 0; i < fitted_entities.size(); ++i)
    {
        const ed::EntityConstPtr& e = fitted_entities[i];
        std::cout << "ERROR for " << e->id() << ": ";

        int n = fitted_entities_num_beams[i];
        if (n < 30)
            std::cout << "not in view" << std::endl;
        else
            std::cout << fitted_entities_errors[i] / n << std::endl;
    }

    // -------------------------------------
    // Filter background

    std::vector<geo::Vec2> sensor_points;
    beam_model_.CalculatePoints(ranges, sensor_points);

    std::vector<geo::Vec2> model_points;
    beam_model_.CalculatePoints(model_ranges, model_points);

    std::vector<double> filtered_ranges(ranges.size(), 0);

    double max_corr_dist = 0.1;
    double max_corr_dist_sq = max_corr_dist * max_corr_dist;
    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double ds = ranges[i];
        double dm = model_ranges[i];

        if (ds <= 0)
            continue;

        if (ds > dm - max_corr_dist)
            continue;

        const geo::Vec2& ps = sensor_points[i];

        // Find the beam window in which possible corresponding points may be situated
        // NOTE: this is an approximation: it underestimates the window size. TODO: fix
        int i_min = std::max(0, beam_model_.CalculateBeam(ps.x - max_corr_dist, ps.y));
        int i_max = std::min(beam_model_.CalculateBeam(ps.x + max_corr_dist, ps.y), (int)ranges.size() - 1);

        // check neighboring points and see if they are within max_corr_dist distance from 'ps'
        bool corresponds = false;
        for(unsigned int j = i_min; j < i_max; ++j)
        {
            const geo::Vec2& pm = model_points[j];
            if (pm.x == pm.x && (ps - pm).length2() < max_corr_dist_sq)
            {
                corresponds = true;
                break;
            }
        }

        if (!corresponds)
            filtered_ranges[i] = ds;
    }

    // -------------------------------------
    // Determine points of interest

    std::vector<Segment> segments;
    segment(filtered_ranges, beam_model_, 0.2, 3, 0.3, segments);

    for(std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const Segment& seg = *it;
        int i_center = seg[seg.size() / 2];
        geo::Vec2 poi_MAP = sensor_pose_xya_2d * beam_model_.CalculatePoint(i_center, ranges[i_center]);

        bool poi_exists = false;
        for(std::vector<geo::Vec2>::const_iterator it_poi = pois_.begin(); it_poi != pois_.end(); ++it_poi)
        {
            if ((poi_MAP - *it_poi).length2() < min_poi_distance_ * min_poi_distance_)
            {
                poi_exists = true;
                break;
            }
        }

        if (!poi_exists)
            pois_.push_back(poi_MAP);
    }

    // -------------------------------------
    // Make snapshot (if requested)

    if (make_snapshot_)
    {
        cv::imshow("Interesting", image->getDepthImage() / 10);

        ed::UUID snapshot_id = ed::Entity::generateID();
        Snapshot& snapshot = snapshots_[snapshot_id];
        snapshot.image = image;
        snapshot.sensor_pose_xya = sensor_pose_xya;
        snapshot.sensor_pose_zrp = sensor_pose_zrp;
        snapshot.canvas = image->getDepthImage();

        ++revision_;
        snapshot.revision = revision_;

        // For easy debugging (auto fitting):
//        ed_sensor_integration::FitModel::Request fit_req;
//        ed_sensor_integration::FitModel::Response fit_res;
//        fit_req.image_id = snapshot_id.str();
//        fit_req.model_name = models_.begin()->first;
//        srvFitModel(fit_req, fit_res);
    }

    // -------------------------------------
    // Visualize

    cv::Mat canvas(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

    DrawWorldVisualization(world, sensor_pose_xya, canvas);
    DrawRanges(ranges,          cv::Scalar(0, 80, 0),  canvas);
    DrawRanges(model_ranges,    cv::Scalar(0, 0, 255), canvas);
    DrawRanges(filtered_ranges, cv::Scalar(0, 255, 0), canvas);

    // Draw segments
    for(std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const Segment& seg = *it;
        geo::Vec2 p1 = beam_model_.CalculatePoint(seg.front(), ranges[seg.front()]);
        geo::Vec2 p2 = beam_model_.CalculatePoint(seg.back(), ranges[seg.back()]);

        cv::Point p1_canvas(p1.x * 100 + canvas.cols / 2, canvas.rows - p1.y * 100);
        cv::Point p2_canvas(p2.x * 100 + canvas.cols / 2, canvas.rows - p2.y * 100);

        cv::line(canvas, p1_canvas, p2_canvas, cv::Scalar(0, 255, 255), 2);
    }

    // Draw points of interest
    for(std::vector<geo::Vec2>::const_iterator it = pois_.begin(); it != pois_.end(); ++it)
    {
        geo::Vec2 p = sensor_pose_xya_2d.inverse() * (*it);
        cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);
        cv::circle(canvas, p_canvas, 3, cv::Scalar(255, 255, 0), 2);
    }

    cv::imshow("rgbd beams", canvas);
    cv::imshow("depth", image->getDepthImage() / 10);
    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::NextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = rgbd_client_.nextImage();
    if (rgbd_image && rgbd_image->getDepthImage().data)
        image_buffer_.push(rgbd_image);

    if (image_buffer_.empty())
        return false;

    rgbd_image = image_buffer_.front();

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("/map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
        image_buffer_.pop();
    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform(root_frame, rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
            {
                image_buffer_.pop();
                ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(rgbd_image->getTimestamp()));
            }

            return false;
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            return false;
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
        return false;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    image = rgbd_image;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::CalculateRanges(const rgbd::Image& image, const geo::Pose3D& sensor_pose_zrp, std::vector<double>& ranges) const
{
    const cv::Mat& depth = image.getDepthImage();
    rgbd::View view(image, depth.cols);

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
            geo::Vector3 p_floor = sensor_pose_zrp * p_sensor;

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

const EntityRepresentation2D* FitterPlugin::GetOrCreateEntity2D(const ed::EntityConstPtr& e)
{
    std::map<ed::UUID, EntityRepresentation2D>::const_iterator it_model = entity_shapes_.find(e->id());
    if (it_model != entity_shapes_.end())
        return &it_model->second;

    // Decompose entity pose into X Y YAW and Z ROLL PITCH
    geo::Pose3D pose_xya;
    geo::Pose3D pose_zrp;
    decomposePose(e->pose(), pose_xya, pose_zrp);

    EntityRepresentation2D& entity_model = entity_shapes_[e->id()];
    dml::project2D(e->shape()->getMesh().getTransformed(pose_zrp), entity_model.shape_2d);

    return &entity_model;
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::RenderEntity(const ed::EntityConstPtr& e, const geo::Pose3D& sensor_pose_xya, int identifier,
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

    const EntityRepresentation2D* e2d = GetOrCreateEntity2D(e);

    geo::Transform2 pose_2d_SENSOR = sensor_pose_xya_2d.inverse() * XYYawToTransform2(pose_xya);

    beam_model_.RenderModel(e2d->shape_2d, pose_2d_SENSOR, identifier, model_ranges, identifiers);
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::updateSnapshots()
{
    for(std::map<ed::UUID, Snapshot>::iterator it = snapshots_.begin(); it != snapshots_.end(); ++it)
    {
        Snapshot& snapshot = it->second;

        bool changed;
        DrawWorldModelOverlay(*snapshot.image, snapshot.sensor_pose_xya * snapshot.sensor_pose_zrp,
                              *world_model_, fitted_entity_ids_, snapshot.canvas, changed);

        if (changed)
        {
            ++revision_;
            snapshot.revision = revision_;
        }
    }

    need_snapshot_update_ = false;
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::DrawWorldVisualization(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, cv::Mat& canvas)
{
    geo::Transform2 sensor_pose_xya_2d = XYYawToTransform2(sensor_pose_xya);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->shape() || !e->has_pose())
            continue;

        // Decompose entity pose into X Y YAW and Z ROLL PITCH
        geo::Pose3D pose_xya;
        geo::Pose3D pose_zrp;
        decomposePose(e->pose(), pose_xya, pose_zrp);

        const EntityRepresentation2D* e2d = GetOrCreateEntity2D(e);

        geo::Transform2 pose_2d_SENSOR = sensor_pose_xya_2d.inverse() * XYYawToTransform2(pose_xya);

        // visualize model
        for(Shape2D::const_iterator it_contour = e2d->shape_2d.begin(); it_contour != e2d->shape_2d.end(); ++it_contour)
        {
            const std::vector<geo::Vec2>& model = *it_contour;
            for(unsigned int i = 0; i < model.size(); ++i)
            {
                unsigned int j = (i + 1) % model.size();
                const geo::Vec2& p1 = pose_2d_SENSOR * model[i];
                const geo::Vec2& p2 = pose_2d_SENSOR * model[j];

                cv::Point p1_canvas(p1.x * 100 + canvas.cols / 2, canvas.rows - p1.y * 100);
                cv::Point p2_canvas(p2.x * 100 + canvas.cols / 2, canvas.rows - p2.y * 100);

                cv::line(canvas, p1_canvas, p2_canvas, cv::Scalar(255, 0, 0), 2);
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::DrawRanges(const std::vector<double>& ranges, const cv::Scalar& color, cv::Mat& canvas)
{
    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double d = ranges[i];
        if (d > 0)
        {
            geo::Vec2 p = beam_model_.CalculatePoint(i, d);
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);
            cv::circle(canvas, p_canvas, 1, color);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvFitModel(ed_sensor_integration::FitModel::Request& req, ed_sensor_integration::FitModel::Response& res)
{
    std::map<ed::UUID, Snapshot>::const_iterator it_image = snapshots_.find(req.image_id);
    if (it_image == snapshots_.end())
    {
        res.error_msg = "Snapshot with ID '" + req.image_id + "' could not be found.";
        return true;
    }

    std::map<std::string, EntityRepresentation2D>::const_iterator it_model = models_.find(req.model_name);
    if (it_model == models_.end())
    {
        res.error_msg = "Model '" + req.model_name + "' could not be found.";
        return true;
    }

    const Snapshot& snapshot = it_image->second;
    const EntityRepresentation2D& model = it_model->second;

    // -------------------------------------
    // Calculate virtual rgbd beam ranges

    std::vector<double> ranges;
    CalculateRanges(*snapshot.image, snapshot.sensor_pose_zrp, ranges);

    // -------------------------------------
    // Render world model objects

    std::vector<double> model_ranges(ranges.size(), 0);
    std::vector<int> dummy_identifiers(ranges.size(), -1);
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        RenderEntity(e, snapshot.sensor_pose_xya, -1, model_ranges, dummy_identifiers);
    }

    // -------------------------------------
    // Fit

//    // Calculate beam number corresponding to click location in image
//    rgbd::View view(*snapshot.image, snapshot.image->getRGBImage().cols);
//    geo::Vec3 click_ray = view.getRasterizer().project2Dto3D(req.click_x, req.click_y);
//    geo::Vec3 p_aligned = snapshot.sensor_pose_zrp * click_ray;
//    int i_click_beam = beam_model_.CalculateBeam(p_aligned.x, p_aligned.y);

//    std::cout << "BEAM: " << i_click_beam << std::endl;

    double min_error = 1e9;
    std::vector<double> best_model_ranges;
    geo::Transform2 best_pose;

    for(int i_beam = 0; i_beam < ranges.size(); ++i_beam)
    {
        double l = beam_model_.rays()[i_beam].length();
        geo::Vec2 r = beam_model_.rays()[i_beam] / l;

        for(double alpha = 0; alpha < 3.1415 * 2; alpha += 0.1)
        {
            // ----------------
            // Calculate rotation

            double cos_alpha = cos(alpha);
            double sin_alpha = sin(alpha);
            geo::Mat2 rot(cos_alpha, -sin_alpha, sin_alpha, cos_alpha);
            geo::Transform2 pose(rot, r * 10);

            // ----------------
            // Determine initial pose based on measured range

            std::vector<double> test_ranges(ranges.size(), 0);
            beam_model_.RenderModel(model.shape_2d, pose, 0, test_ranges, dummy_identifiers);

            double ds = ranges[i_beam];
            double dm = test_ranges[i_beam];

            if (ds <= 0 || dm <= 0)
                continue;

            pose.t += r * ((ds - dm) * l);

            // ----------------
            // Render model

            test_ranges = model_ranges;
            beam_model_.RenderModel(model.shape_2d, pose, 0, test_ranges, dummy_identifiers);

            // ----------------
            // Calculate error

            int n = 0;
            double total_error = 0;
            for(unsigned int i = 0; i < test_ranges.size(); ++i)
            {
                double ds = ranges[i];
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
                best_model_ranges = test_ranges;
                best_pose = pose;
                min_error = error;
            }
        }
    }

    std::cout << "Found a pose: " << best_pose << std::endl;

    // -------------------------------------
    // Add entity

    // Determine ID
    ed::UUID new_id;
    for(unsigned int i = 0; true; ++i)
    {
        std::stringstream ss;
        ss << req.model_name << "-" << i;
        new_id = ss.str();

        if (!world_model_->getEntity(new_id))
            break;
    }

    std::stringstream error;
    model_loader_.create(new_id, req.model_name, *update_request_, error);

    geo::Pose3D pose_3d;
    pose_3d.t = geo::Vec3(best_pose.t.x, best_pose.t.y, 0);
    pose_3d.R = geo::Mat3::identity();
    pose_3d.R.xx = best_pose.R.xx;
    pose_3d.R.xy = best_pose.R.xy;
    pose_3d.R.yx = best_pose.R.yx;
    pose_3d.R.yy = best_pose.R.yy;

    update_request_->setPose(new_id, snapshot.sensor_pose_xya * pose_3d);
    update_request_->setType(new_id, req.model_name);
    update_request_->setFlag(new_id, "furniture");

    need_snapshot_update_ = true;  // Indicates that on next plugin cycle the snapshots need to be updated
    fitted_entity_ids_.insert(new_id);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvGetModels(ed_sensor_integration::GetModels::Request& req, ed_sensor_integration::GetModels::Response& res)
{
    res.model_names.resize(models_.size());
    res.model_images.resize(models_.size());

    unsigned int i = 0;
    for(std::map<std::string, EntityRepresentation2D>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        res.model_names[i] = model_name;

        cv::Mat img(200, 200, CV_8UC3, cv::Scalar(100, 100, 100));
        cv::putText(img, model_name, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.4, cv::Scalar(0, 0, 255), 2);

        ImageToMsg(img, "jpg", res.model_images[i]);

        ++i;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvGetSnapshots(ed_sensor_integration::GetSnapshots::Request& req, ed_sensor_integration::GetSnapshots::Response& res)
{
    if (req.revision >= revision_)
    {
        res.new_revision = revision_;
        return true;
    }

    for(std::map<ed::UUID, Snapshot>::const_iterator it = snapshots_.begin(); it != snapshots_.end(); ++it)
    {
        const Snapshot& snapshot = it->second;
        if (snapshot.revision <= req.revision)
            continue;

        // Add snapshot image
        res.images.push_back(ed_sensor_integration::ImageBinary());
        ed_sensor_integration::ImageBinary& img_msg = res.images.back();
        ImageToMsg(snapshot.canvas, "jpg", img_msg);

        // Add snapshot id
        res.image_ids.push_back(it->first.str());
    }

    res.new_revision = revision_;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvMakeSnapshot(ed_sensor_integration::MakeSnapshot::Request& req, ed_sensor_integration::MakeSnapshot::Response& res)
{
    make_snapshot_ = true;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvGetPOIs(ed_sensor_integration::GetPOIs::Request& req, ed_sensor_integration::GetPOIs::Response& res)
{
    res.pois.resize(pois_.size());
    for(unsigned int i = 0; i < pois_.size(); ++i)
    {
        geometry_msgs::PointStamped& p = res.pois[i];
        p.header.frame_id = "/map";
        p.point.x = pois_[i].x;
        p.point.y = pois_[i].y;
        p.point.z = 0;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(FitterPlugin)

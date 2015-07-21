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
        rgb_params[1] = 50; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", rgb_image, msg.data, rgb_params))
        {
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

FitterPlugin::FitterPlugin() : tf_listener_(0), revision_(0)
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

    ros::NodeHandle nh_global;
    nh_global.setCallbackQueue(&cb_queue_);
    std::string nav_goal_topic;
    if (config.value("nav_goal_topic", nav_goal_topic))
        navigator_.initialize(nh_global, nav_goal_topic);

    std::string map_topic_in, map_topic_out;
    if (config.value("map_topic_in", map_topic_in))
    {
        config.value("map_topic_out", map_topic_out);
        map_filter_.initialize(map_topic_in, map_topic_out);
    }

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

            cv::Mat model_image;
            std::string image_path;
            if (config.value("image", image_path, tue::OPTIONAL))
            {
                model_image = cv::imread(image_path);

                if (!model_image.data)
                    ed::log::error() << "Could not load model image: '" << image_path << "'." << std::endl;
            }

            if (!model_image.data)
            {
                model_image = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
            }

            geo::ShapeConstPtr shape = it_shape->second;

            EntityRepresentation2D& model = models_[name];
            model.model_image = model_image;
            dml::project2D(shape->getMesh(), model.shape_2d);
        }

        config.endArray();
    }

    beam_model_.initialize(4, 200);

    tf_listener_ = new tf::TransformListener;

    // Initialize services
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_fit_model_ = nh.advertiseService("gui/fit_model", &FitterPlugin::srvFitModel, this);
    srv_get_models_ = nh.advertiseService("gui/get_models", &FitterPlugin::srvGetModels, this);
    srv_get_snapshots_ = nh.advertiseService("gui/get_snapshots", &FitterPlugin::srvGetSnapshots, this);
    srv_make_snapshot_ = nh.advertiseService("make_snapshot", &FitterPlugin::srvMakeSnapshot, this);
    srv_get_pois_ = nh.advertiseService("get_pois", &FitterPlugin::srvGetPOIs, this);
    srv_navigate_to_ = nh.advertiseService("navigate_to", &FitterPlugin::srvNavigateTo, this);
    srv_create_walls_ = nh.advertiseService("create_walls", &FitterPlugin::srvCreateWalls, this);


    // Visualization
    debug_viz_.initialize("viz/fitter");
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    world_model_ = &world;
    update_request_ = &req;

    // -------------------------------------
    // Update snapshots if needed

    if (!changed_entity_ids_.empty())
        updateSnapshots();

    // -------------------------------------
    // Handle service requests

    make_snapshot_ = false;
    cb_queue_.callAvailable();

    // -------------------------------------
    // Map filter

//    map_filter_.update();

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

    Snapshot& current_image = snapshots_["current"];
    current_image.image = image;
    current_image.sensor_pose_xya = sensor_pose_xya;
    current_image.sensor_pose_zrp = sensor_pose_zrp;

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
        if (fitted_entity_ids_.find(e->id()) == fitted_entity_ids_.end() && e->id() != "walls")
            RenderEntity(e, sensor_pose_xya, -1, model_ranges_background, rendered_indices);
    }

    // -------------------------------------
    // Render fitted objects

    std::vector<ed::EntityConstPtr> entities_to_check;

    std::vector<double> model_ranges = model_ranges_background;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (fitted_entity_ids_.find(e->id()) != fitted_entity_ids_.end())
        {
            int identifier = -1;
            if (e->hasFlag("dynamic"))
            {
                identifier = entities_to_check.size();
                entities_to_check.push_back(e);

                // Just for now (Hack!)
                update_request_->removeFlag(e->id(), "dynamic");
            }

            RenderEntity(e, sensor_pose_xya, identifier, model_ranges, rendered_indices);
        }
    }

    // -------------------------------------
    // Determine position errors of fitted objects

    if (!entities_to_check.empty())
    {
        std::vector<int> fitted_entities_num_beams(entities_to_check.size(), 0); // number of beams corresponding to this entity
        std::vector<double> fitted_entities_errors(entities_to_check.size(), 0);    // error corresponding to this entity

        for(unsigned int i = 0; i < ranges.size(); ++i)
        {
            double rs = ranges[i];
            if (rs == 0)
                continue;

            int i_entity = rendered_indices[i];
            if (i_entity < 0) // Does not belong to any fitted entity
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

        // -------------------------------------
        // Re-fit incorrect positioned objects

        for(unsigned int i = 0; i < entities_to_check.size(); ++i)
        {
            const ed::EntityConstPtr& e = entities_to_check[i];

            // Calculate 2d entity pose in sensor frame
            geo::Pose3D e_pose_xya;
            geo::Pose3D e_pose_zrp;
            decomposePose(e->pose(), e_pose_xya, e_pose_zrp);
            geo::Transform2 e_pose_2d_SENSOR = sensor_pose_xya_2d.inverse() * XYYawToTransform2(e_pose_xya);

            double dist_to_entity = e_pose_2d_SENSOR.t.length();
            int n = fitted_entities_num_beams[i];
            double err = fitted_entities_errors[i] / n;

//            std::cout << "Checking " << e->id() << ": distance = " << dist_to_entity << ", num_beams = " << n << ", error = " << err << std::endl;

            if (dist_to_entity > 4 || dist_to_entity < 1.5) // TODO: hard-coded value
                continue;

            if (n < 30 || err < 0.1)   // TODO: hard-coded
                continue;  // Error not big or entity not fully in view, so don't update it

            const EntityRepresentation2D* model = GetOrCreateEntity2D(e);

            int i_beam = beam_model_.CalculateBeam(e_pose_2d_SENSOR.t.x, e_pose_2d_SENSOR.t.y);
            if (i_beam < 0 || i_beam >= ranges.size())
                continue;

            int beam_search_window = 100; // TODO: hard-coded

            geo::Pose3D new_pose;
            if (FitEntity(e->id(), i_beam, beam_search_window, model->shape_2d, ranges, sensor_pose_xya, new_pose, false))
            {
                update_request_->setPose(e->id(), new_pose);
                changed_entity_ids_.insert(e->id());
            }
        }

        // TODO: re-render world model with re-fitted objects (for better background filter)
    }

//    // -------------------------------------
//    // Filter background

//    std::vector<geo::Vec2> sensor_points;
//    beam_model_.CalculatePoints(ranges, sensor_points);

//    std::vector<geo::Vec2> model_points;
//    beam_model_.CalculatePoints(model_ranges, model_points);

//    std::vector<double> filtered_ranges(ranges.size(), 0);

//    double max_corr_dist = 0.1;
//    double max_corr_dist_sq = max_corr_dist * max_corr_dist;
//    for(unsigned int i = 0; i < ranges.size(); ++i)
//    {
//        double ds = ranges[i];
//        double dm = model_ranges[i];

//        if (ds <= 0)
//            continue;

//        if (ds > dm - max_corr_dist)
//            continue;

//        const geo::Vec2& ps = sensor_points[i];

//        // Find the beam window in which possible corresponding points may be situated
//        // NOTE: this is an approximation: it underestimates the window size. TODO: fix
//        int i_min = std::max(0, beam_model_.CalculateBeam(ps.x - max_corr_dist, ps.y));
//        int i_max = std::min(beam_model_.CalculateBeam(ps.x + max_corr_dist, ps.y), (int)ranges.size() - 1);

//        // check neighboring points and see if they are within max_corr_dist distance from 'ps'
//        bool corresponds = false;
//        for(unsigned int j = i_min; j < i_max; ++j)
//        {
//            const geo::Vec2& pm = model_points[j];
//            if (pm.x == pm.x && (ps - pm).length2() < max_corr_dist_sq)
//            {
//                corresponds = true;
//                break;
//            }
//        }

//        if (!corresponds)
//            filtered_ranges[i] = ds;
//    }

//    // -------------------------------------
//    // Determine points of interest

//    std::vector<Segment> segments;
//    segment(filtered_ranges, beam_model_, 0.2, 3, 0.3, segments);

//    for(std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
//    {
//        const Segment& seg = *it;
//        int i_center = seg[seg.size() / 2];
//        geo::Vec2 poi_MAP = sensor_pose_xya_2d * beam_model_.CalculatePoint(i_center, ranges[i_center]);

//        bool poi_exists = false;
//        for(std::vector<geo::Vec2>::const_iterator it_poi = pois_.begin(); it_poi != pois_.end(); ++it_poi)
//        {
//            if ((poi_MAP - *it_poi).length2() < min_poi_distance_ * min_poi_distance_)
//            {
//                poi_exists = true;
//                break;
//            }
//        }

//        if (!poi_exists)
//            pois_.push_back(poi_MAP);
//    }

    // -------------------------------------
    // Make snapshot (if requested)

    if (make_snapshot_)
    {
        ed::UUID snapshot_id = ed::Entity::generateID();
        Snapshot& snapshot = snapshots_[snapshot_id];
        snapshot.image = image;
        snapshot.sensor_pose_xya = sensor_pose_xya;
        snapshot.sensor_pose_zrp = sensor_pose_zrp;
        snapshot.first_timestamp = image->getTimestamp();

        double h = 640;
        double w = (h * image->getRGBImage().rows) / image->getRGBImage().cols;

        cv::resize(image->getRGBImage(), snapshot.background_image, cv::Size(h, w));

        bool changed;
        DrawWorldModelOverlay(world, fitted_entity_ids_, fitted_entity_ids_, snapshot, true, changed);

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

    if (debug_viz_.enabled())
    {
        cv::Mat canvas(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

        DrawWorldVisualization(world, sensor_pose_xya, canvas);
        DrawRanges(ranges,          cv::Scalar(0, 80, 0),  canvas);
        DrawRanges(model_ranges,    cv::Scalar(0, 0, 255), canvas);
//        DrawRanges(filtered_ranges, cv::Scalar(0, 255, 0), canvas);

//        // Draw segments
//        for(std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
//        {
//            const Segment& seg = *it;
//            geo::Vec2 p1 = beam_model_.CalculatePoint(seg.front(), ranges[seg.front()]);
//            geo::Vec2 p2 = beam_model_.CalculatePoint(seg.back(), ranges[seg.back()]);

//            cv::Point p1_canvas(p1.x * 100 + canvas.cols / 2, canvas.rows - p1.y * 100);
//            cv::Point p2_canvas(p2.x * 100 + canvas.cols / 2, canvas.rows - p2.y * 100);

//            cv::line(canvas, p1_canvas, p2_canvas, cv::Scalar(0, 255, 255), 2);
//        }

//        // Draw points of interest
//        for(std::vector<geo::Vec2>::const_iterator it = pois_.begin(); it != pois_.end(); ++it)
//        {
//            geo::Vec2 p = sensor_pose_xya_2d.inverse() * (*it);
//            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);
//            cv::circle(canvas, p_canvas, 3, cv::Scalar(255, 255, 0), 2);
//        }

        debug_viz_.publish(canvas);
    }
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

bool FitterPlugin::FitEntity(const ed::UUID& id, int expected_center_beam, int beam_window, const Shape2D& shape2d,
               const std::vector<double>& sensor_ranges, const geo::Pose3D& sensor_pose_xya,
               geo::Pose3D& expected_pose, bool snap_to_expected_beam)
{
    // -------------------------------------
    // Render world model objects

    std::vector<double> model_ranges(sensor_ranges.size(), 0);
    std::vector<int> dummy_identifiers(sensor_ranges.size(), -1);
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->id() == id || e->id() == "walls") // Skip entity id that needs to be fitted
            continue;

        RenderEntity(e, sensor_pose_xya, -1, model_ranges, dummy_identifiers);
    }

    // -------------------------------------
    // Fit

    double min_error = 1e9;
    geo::Transform2 best_pose;

    std::cout << "expected_center_beam: " << expected_center_beam << std::endl;

    for(int i = 0; i < 2 * beam_window; ++i)
    {
        // Calculate beam: start at 'expected_center_beam' and go left and right increasingly
        int i_beam = expected_center_beam + (i % 2 == 0 ? -1 : 1) * (i / 2);
        if (i_beam < 0 || i_beam >= sensor_ranges.size())
            continue;

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

            std::vector<double> test_ranges(sensor_ranges.size(), 0);
            beam_model_.RenderModel(shape2d, pose, 0, test_ranges, dummy_identifiers);

            double ds = sensor_ranges[i_beam];
            double dm = test_ranges[i_beam];

            if (ds <= 0 || dm <= 0)
                continue;

            pose.t += r * ((ds - dm) * l);

            // ----------------
            // Render model

            test_ranges = model_ranges;
            std::vector<int> identifiers(sensor_ranges.size(), 0);
            beam_model_.RenderModel(shape2d, pose, 1, test_ranges, identifiers);

            if (snap_to_expected_beam && identifiers[expected_center_beam] != 1)  // expected center beam MUST contain the rendered model
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
                best_pose = pose;
                min_error = error;
            }
        }
    }

    if (min_error > 1e5)
    {
        std::cout << "No pose found!" << std::endl;
        return false;
    }

    std::cout << "Found a pose: " << best_pose << std::endl;

    // Update map filter
//    map_filter_.setEntityPose(best_pose, shape2d);

    // Convert to 3D Pose

    geo::Pose3D pose_3d;
    pose_3d.t = geo::Vec3(best_pose.t.x, best_pose.t.y, 0);
    pose_3d.R = geo::Mat3::identity();
    pose_3d.R.xx = best_pose.R.xx;
    pose_3d.R.xy = best_pose.R.xy;
    pose_3d.R.yx = best_pose.R.yx;
    pose_3d.R.yy = best_pose.R.yy;

    expected_pose = sensor_pose_xya * pose_3d;

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
    // First update 'snapshot_id_to_first_update_'
    std::map<ed::UUID, Snapshot>::iterator it = snapshots_.find(snapshot_id_to_first_update_);
    if (it != snapshots_.end())
    {
        Snapshot& snapshot = it->second;

        bool changed;
        DrawWorldModelOverlay(*world_model_, fitted_entity_ids_, changed_entity_ids_, snapshot, false, changed);

        if (changed)
        {
            ++revision_;
            snapshot.revision = revision_;
        }
    }

    // Then update the rest
    for(std::map<ed::UUID, Snapshot>::iterator it = snapshots_.begin(); it != snapshots_.end(); ++it)
    {
        Snapshot& snapshot = it->second;

        // Skip snapshot that was already updated
        if (it->first.str() == snapshot_id_to_first_update_)
            continue;

        bool changed;
        DrawWorldModelOverlay(*world_model_, fitted_entity_ids_, changed_entity_ids_, snapshot, false, changed);

        if (changed)
        {
            ++revision_;
            snapshot.revision = revision_;
        }
    }

    changed_entity_ids_.clear();
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
//    // TEMP!!!!
//    ed_sensor_integration::NavigateTo::Request nav_req;
//    ed_sensor_integration::NavigateTo::Response nav_res;

//    nav_req.click_x_ratio = req.click_x_ratio;
//    nav_req.click_y_ratio = req.click_y_ratio;
//    nav_req.snapshot_id = req.image_id;

//    srvNavigateTo(nav_req, nav_res);
//    return true;


    // Only called if 'undo latest fit' is requested
    if (req.undo_latest_fit)
    {
        if (fitted_entity_ids_stack_.empty())
            return true;

        ed::UUID undo_id = fitted_entity_ids_stack_.back();

        fitted_entity_ids_.erase(undo_id);
        update_request_->removeEntity(undo_id);

        // Snapshots need to be redrawn
        changed_entity_ids_.insert(undo_id);
        snapshot_id_to_first_update_ = req.image_id;

        fitted_entity_ids_stack_.pop_back();

        return true;
    }

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
    // Determine beam number corresponding to click location in image
//    int i_click_beam = ranges.size() / 2; // TODO

    std::cout << "Pixel ratios: " << req.click_x_ratio << ", " << req.click_y_ratio << std::endl;

    int click_x = req.click_x_ratio * snapshot.canvas.cols;
    int click_y = req.click_y_ratio * snapshot.canvas.rows;

    std::cout << "Revised coordinates: " << click_x << ", " << click_y << std::endl;

    // Calculate beam number corresponding to click location in image
    rgbd::View view(*snapshot.image, snapshot.canvas.cols);
    geo::Vec3 click_ray = view.getRasterizer().project2Dto3D(click_x, click_y);
    geo::Vec3 p_aligned = snapshot.sensor_pose_zrp * click_ray;
    int i_click_beam = beam_model_.CalculateBeam(p_aligned.x, p_aligned.y);

    std::cout << "i_click_beam = " << i_click_beam << std::endl;



    // -------------------------------------
    // Determine new, unique ID

    ed::UUID new_id;
    for(unsigned int i = 0; true; ++i)
    {
        std::stringstream ss;
        ss << req.model_name << "-" << i;
        new_id = ss.str();

        if (!world_model_->getEntity(new_id))
            break;
    }

    // -------------------------------------
    // Fit

    geo::Pose3D expected_pose;
    if (!FitEntity(new_id, i_click_beam, ranges.size() / 2, model.shape_2d, ranges, snapshot.sensor_pose_xya, expected_pose))
    {
        res.error_msg = "Could not fit model at designated location";
        return true;
    }

    // -------------------------------------
    // Add entity

    // Create entity from model name
    std::stringstream error;
    model_loader_.create(new_id, req.model_name, *update_request_, error);

    std::cout << "Setting " << new_id << "(type: " << req.model_name << ") to " << expected_pose << std::endl;

    update_request_->setPose(new_id, expected_pose);
    update_request_->setType(new_id, req.model_name);
    update_request_->setFlag(new_id, "furniture");

    changed_entity_ids_.insert(new_id);  // Indicates that on next plugin cycle the snapshots need to be updated
    fitted_entity_ids_.insert(new_id);
    fitted_entity_ids_stack_.push_back(new_id);
    snapshot_id_to_first_update_ = req.image_id;

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
        const EntityRepresentation2D& model = it->second;

        res.model_names[i] = model_name;

        ImageToMsg(model.model_image, "jpg", res.model_images[i]);

        ++i;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvGetSnapshots(ed_sensor_integration::GetSnapshots::Request& req, ed_sensor_integration::GetSnapshots::Response& res)
{
    // Remove snapshots that are no longer needed
    for(std::vector<std::string>::const_iterator it = req.delete_ids.begin(); it != req.delete_ids.end(); ++it)
    {
        std::cout << "ERASE : " << *it << std::endl;
        snapshots_.erase(*it);
    }

    if (snapshots_.find("current") != snapshots_.end())
    {
        ros::Time time = ros::Time::now();
        if ((time - last_image_update_) > ros::Duration(0.5))
        {
            Snapshot& current_image = snapshots_["current"];

            double h = 640;
            double w = (h * current_image.image->getRGBImage().rows) / current_image.image->getRGBImage().cols;

            cv::resize(current_image.image->getRGBImage(), current_image.background_image, cv::Size(h, w));
            current_image.canvas = current_image.background_image;
            current_image.visualized_ids.clear();

            bool changed;
            DrawWorldModelOverlay(*world_model_, fitted_entity_ids_, fitted_entity_ids_, current_image, true, changed);

            last_image_update_ = time;

            ++revision_;
            current_image.revision = revision_;
        }
    }

    if (req.revision >= revision_)
    {
        res.new_revision = revision_;
        return true;
    }

    for(std::map<ed::UUID, Snapshot>::const_iterator it = snapshots_.begin(); it != snapshots_.end(); ++it)
    {
        const Snapshot& snapshot = it->second;

        // Check if the client already has the newest version of the snapshot
        if (snapshot.revision <= req.revision)
            continue;

        // Check if the client
        if (req.max_num_revisions > 0 && snapshot.revision > req.revision + req.max_num_revisions)
            continue;

        // Add snapshot image
        res.images.push_back(ed_sensor_integration::ImageBinary());
        ed_sensor_integration::ImageBinary& img_msg = res.images.back();
        ImageToMsg(snapshot.canvas, "jpg", img_msg);

        // Add snapshot id
        res.image_ids.push_back(it->first.str());

        // Add snapshot timestamp
        res.image_timestamps.push_back(ros::Time(snapshot.first_timestamp));
    }

    if (req.max_num_revisions == 0)
        res.new_revision = revision_;
    else
        res.new_revision = std::min(revision_, req.revision + req.max_num_revisions);

    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->hasFlag("furniture"))
            res.entity_ids.push_back(e->id().str());
    }

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
    if (req.reset_pois)
    {
        pois_.clear();
        snapshots_.clear();
        changed_entity_ids_.clear();
        fitted_entity_ids_.clear();
        fitted_entity_ids_stack_.clear();
        return true;
    }

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

bool FitterPlugin::srvNavigateTo(ed_sensor_integration::NavigateTo::Request& req, ed_sensor_integration::NavigateTo::Response& res)
{
    std::map<ed::UUID, Snapshot>::const_iterator it_image = snapshots_.find(req.snapshot_id);
    if (it_image == snapshots_.end())
    {
        res.error_msg = "Snapshot with ID '" + req.snapshot_id + "' could not be found.";
        return true;
    }

    const Snapshot& snapshot = it_image->second;

    const cv::Mat& depth = snapshot.image->getDepthImage();

    int click_x = req.click_x_ratio * depth.cols;
    int click_y = req.click_y_ratio * depth.rows;

    if (!navigator_.navigate(snapshot, click_x, click_y))
    {
        res.error_msg = "Could not navigate to point";
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool FitterPlugin::srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    geo::ShapeConstPtr shape = map_filter_.createWallShape(1.5);

    if (shape)
    {
        ed::UUID id = "walls";

        update_request_->setShape(id, shape);
        update_request_->setPose(id, geo::Pose3D::identity());
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(FitterPlugin)

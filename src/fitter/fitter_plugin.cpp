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

    if (encoding == "jpg")
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", image, msg.data, rgb_params)) {
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

        if (!cv::imencode(".png", image, msg.data, params)) {
            std::cout << "PNG image compression failed" << std::endl;
            return false;
        }
    }

    return true;
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
}

// ----------------------------------------------------------------------------------------------------

void FitterPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    // -------------------------------------
    // Handle service requests

    world_model_ = &world;
    update_request_ = &req;
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

    // -------------------------------------
    // Calculate virtual rgbd beam ranges

    std::vector<double> ranges;
    CalculateRanges(*image, sensor_pose_zrp, ranges);

    // -------------------------------------
    // Render world model objects

    std::vector<double> model_ranges;
    RenderWorldModel(world, sensor_pose_xya, model_ranges);

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
    // Determine snapshot: decide whether this is an interesting image

    unsigned int num_beams = ranges.size();

    unsigned int i_interesting_min = 0.3 * num_beams;
    unsigned int i_interesting_max = 0.7 * num_beams;

    int n_interesting_points = 0;
    for(unsigned int i = i_interesting_min; i < i_interesting_max; ++i)
    {
        double d = filtered_ranges[i];
        if (d > 0 && d < 4)
            ++n_interesting_points;
    }

    if ((double)n_interesting_points / (i_interesting_max - i_interesting_min) > 0.8)
    {
        // Interesting picture. Let's see if we already created a similar shot (check sensor origin and orientation)

        bool similar = false;
        for(std::map<ed::UUID, Snapshot>::const_iterator it = snapshots_.begin(); it != snapshots_.end(); ++it)
        {
            const Snapshot& snapshot = it->second;

            double diff_t_sq = (snapshot.sensor_pose_xya.t - sensor_pose_xya.t).length2();
            double diff_roi_sq = (snapshot.sensor_pose_xya * geo::Vec3(2, 0, 0) - sensor_pose_xya * geo::Vec3(2, 0, 0)).length2();

            if (diff_t_sq < 2 * 2 && diff_roi_sq < 1 * 1)
            {
                similar = true;
                break;
            }
        }

        if (!similar)
        {
            cv::imshow("Interesting", image->getDepthImage() / 10);

            ed::UUID snapshot_id = ed::Entity::generateID();
            Snapshot& snapshot = snapshots_[snapshot_id];
            snapshot.image = image;
            snapshot.sensor_pose_xya = sensor_pose_xya;
            snapshot.sensor_pose_zrp = sensor_pose_zrp;

            ++revision_;
            snapshot.revision = revision_;

            std::cout << "INTERESTING! " << revision_ << std::endl;
        }
    }

    // -------------------------------------
    // Visualize

    cv::Mat canvas(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

    DrawWorldVisualization(world, sensor_pose_xya, canvas);
    DrawRanges(ranges,          cv::Scalar(0, 80, 0),  canvas);
    DrawRanges(model_ranges,    cv::Scalar(0, 0, 255), canvas);
    DrawRanges(filtered_ranges, cv::Scalar(0, 255, 0), canvas);

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

void FitterPlugin::RenderWorldModel(const ed::WorldModel& world, const geo::Pose3D& sensor_pose_xya, std::vector<double>& model_ranges)
{
    geo::Transform2 sensor_pose_xya_2d = XYYawToTransform2(sensor_pose_xya);

    if (model_ranges.size() != beam_model_.num_beams())
        model_ranges.resize(beam_model_.num_beams(), 0);

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

        beam_model_.RenderModel(e2d->shape_2d, pose_2d_SENSOR, model_ranges);
    }
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

    std::vector<double> model_ranges;
    RenderWorldModel(*world_model_, snapshot.sensor_pose_xya, model_ranges);

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
            beam_model_.RenderModel(model.shape_2d, pose, test_ranges);

            double ds = ranges[i_beam];
            double dm = test_ranges[i_beam];

            if (ds <= 0 || dm <= 0)
                continue;

            pose.t += r * ((ds - dm) * l);

            // ----------------
            // Render model

            test_ranges = model_ranges;
            beam_model_.RenderModel(model.shape_2d, pose, test_ranges);

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
        ImageToMsg(snapshot.image->getDepthImage(), "jpg", img_msg);

        // Add snapshot id
        res.image_ids.push_back(it->first.str());
    }

    res.new_revision = revision_;

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(FitterPlugin)

#include "plugin.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

// For copy-pasted kinect sensor module code
// -------------------------------------------
#include <ed/types.h>
#include <ed/measurement.h>
#include <ed/rgbd_data.h>
#include <ed/helpers/depth_data_processing.h>
#include <ed/helpers/visualization.h>
#include "ed_sensor_integration/kinect/almodules/rgbd_al_module.h"
#include <ed/update_request.h>
#include "ed_sensor_integration/kinect/almodules/point_normal_alm.h"
#include "ed_sensor_integration/kinect/almodules/polygon_height_alm.h"
// -------------------------------------------
// END For copy-pasted kinect sensor module code

#include "ed/segmentation_modules/euclidean_clustering_sm.h"

#include <opencv2/highgui/highgui.hpp>

#include <geolib/ros/tf_conversions.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), in_view(false) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            in_view = true;
            z_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat& z_buffer;
    bool in_view;

};

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::configure(tue::Configuration config)
{
    std::string topic;
    if (config.value("topic", topic))
        kinect_client_.intialize(topic);

    // - - - - - - - - - - - - - - - - - -
    // Load tunable parameters

    config.value("voxel_size", voxel_size_);
    config.value("max_range", max_range_);
    config.value("clearing_padding_fraction", clearing_padding_fraction_);
    config.value("normal_k_search", normal_k_search_);

    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image

    rgbd::ImagePtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
        return;


    // - - - - - - - - - - - - - - - - - -
    // Determine absolte kinect pose based on TF

    geo::Pose3D sensor_pose;

    if (!tf_listener_->waitForTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(0.5)))
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose");
        return;
    }

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);


    // - - - - - - - - - - - - - - - - - -
    // Render world model based on pose calculated above

    rgbd::View view(*rgbd_image, 160);

    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape())
        {
            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            SampleRenderResult res(model);

            // Render
            rasterizer.render(opt, res);

            if (res.in_view)
                entities.push_back(e);
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try other poses and determine best scoring pose

    cv::Mat best_model;
    double min_error = 1e9; // TODO
    geo::Pose3D best_pose;

    for(double dx = -0.2; dx < 0.2; dx += 0.05)
    {
        for(double dy = -0.2; dy < 0.2; dy += 0.05)
        {
            for(double da = -0.1; da < 0.1; da += 0.05)
            {
                geo::Matrix3 m;
                m.setRPY(0, 0, da);

                geo::Pose3D test_pose;
                test_pose.t = sensor_pose.t + geo::Vector3(dx, dy, 0);
                test_pose.R = m * sensor_pose.R;

                // Render world
                cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
                for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
                {
                    const ed::EntityConstPtr& e = *it;

                    geo::Pose3D pose = test_pose.inverse() * e->pose();
                    geo::RenderOptions opt;
                    opt.setMesh(e->shape()->getMesh(), pose);

                    SampleRenderResult res(model);

                    // Render
                    rasterizer.render(opt, res);
                }

                int n = 0;
                double total_error = 0;
                for(int y = 0; y < view.getHeight(); ++y)
                {
                    for(int x = 0; x < view.getWidth(); ++x)
                    {
                        float dm = model.at<float>(y, x);
                        float ds = view.getDepth(x, y);

                        if (dm > 0 && ds > 0) // TODO
                        {
                            if (std::abs(dm - ds) > 0.05)
                                total_error += 1;
                            ++n;
                        }
                    }
                }

                if (n > 0)
                {
                    double avg_error = total_error / n;
                    if (avg_error < min_error)
                    {
                        min_error = avg_error;
                        best_pose = test_pose;
                        best_model = model;
                    }
                }


            }
        }
    }


    // - - - - - - - - - - - - - - - - - -
    // Calculate and show diff

    cv::Mat diff(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);

    for(int y = 0; y < view.getHeight(); ++y)
    {
        for(int x = 0; x < view.getWidth(); ++x)
        {
            float dm = best_model.at<float>(y, x);
            float ds = view.getDepth(x, y);

            if (dm > 0 && ds > 0)
            {
                float err = std::abs(dm - ds);
                if (err > 0.05)
                    diff.at<float>(y, x) = err;
            }
        }
    }

//    cv::imshow("depth", rgbd_image->getDepthImage() / 8);
//    cv::imshow("initial model", model / 8);
//    cv::imshow("best model", best_model / 8);
//    cv::imshow("diff", diff);
//    cv::waitKey(3);


    // - - - - - - - - - - - - - - - - - -
    // Create point cloud from rgbd data

    ed::RGBDData rgbd_data;
    rgbd_data.image = rgbd_image;
    rgbd_data.sensor_pose = best_pose;

    ed::helpers::ddp::extractPointCloud(rgbd_data, voxel_size_, max_range_, 1);
    ed::helpers::ddp::calculatePointCloudNormals(rgbd_data, normal_k_search_);

    ros::NodeHandle nh("~/kinect_plugin");
    ros::Publisher vis_marker_pub = nh.advertise<visualization_msgs::Marker>("vis_markers",0);
    ed::helpers::visualization::publishPclVisualizationMarker(best_pose,rgbd_data.point_cloud,vis_marker_pub, 0, "sensor_data_best_pose");


    // - - - - - - - - - - - - - - - - - -
    // Association

    ed::PointCloudMaskPtr pc_mask(new ed::PointCloudMask(rgbd_data.point_cloud->points.size()));
    for(unsigned int i = 0; i < pc_mask->size(); ++i)
        (*pc_mask)[i] = i;

    edKinect::ALMResult alm_result;

    // process data using hard coded modules
    point_normal_alm_.process(rgbd_data,pc_mask,world,alm_result);
    polygon_height_alm_.process(rgbd_data,pc_mask,world,alm_result);

    for(std::map<ed::UUID, std::vector<ed::MeasurementConstPtr> >::const_iterator it = alm_result.associations.begin(); it != alm_result.associations.end(); ++it)
    {
        const std::vector<ed::MeasurementConstPtr>& measurements = it->second;
        req.addMeasurements(it->first, measurements);
    }


    // - - - - - - - - - - - - - - - - - -
    // Segmentation of residual sensor data

    if (!pc_mask->empty())
    {
        std::vector<ed::PointCloudMaskPtr> segments;
        segments.push_back(pc_mask);

        euclidean_clustering_sm_.process(rgbd_data, segments);

        for (std::vector<ed::PointCloudMaskPtr>::const_iterator it = segments.begin(); it != segments.end(); ++it)
        {
            ed::MeasurementConstPtr m(new ed::Measurement(rgbd_data, *it));
            req.addMeasurement(ed::Entity::generateID(), m);
        }

    }


    // - - - - - - - - - - - - - - - - - -
    // Clearing

    std::vector<ed::UUID> entities_in_view_not_associated;
    for (ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->shape())
        {
            bool in_frustrum, object_in_front;
            if (ed::helpers::ddp::inView(rgbd_data.image, rgbd_data.sensor_pose, e->convexHull().center_point, max_range_, clearing_padding_fraction_, in_frustrum, object_in_front))
            {
                ed::MeasurementConstPtr m = e->lastMeasurement();

                if (m && ros::Time::now().toSec() - m->timestamp() > 1.0)
                {
                    entities_in_view_not_associated.push_back(e->id());
                }
            }
        }
    }

    for (std::vector<ed::UUID>::const_iterator it = entities_in_view_not_associated.begin(); it != entities_in_view_not_associated.end(); ++it)
        req.removeEntity(*it);

}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

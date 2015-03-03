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

// Localization
#include "sampling_render_localizer.h"
#include "sampling_projector_localizer.h"

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
    if (config.value("topic", topic_))
        kinect_client_.intialize(topic_);

    // - - - - - - - - - - - - - - - - - -
    // Load tunable parameters

    config.value("voxel_size", voxel_size_);
    config.value("max_range", max_range_);
    config.value("clearing_padding_fraction", clearing_padding_fraction_);
    config.value("normal_k_search", normal_k_search_);
    config.value("visualize",visualize_);

    std::cout << "Parameters kinect plugin: \n" <<
                 "- voxel size: " << voxel_size_ << "\n" <<
                 "- max_range_: " << max_range_ << "\n" <<
                 "- clearing_padding_fraction_: " << clearing_padding_fraction_ << "\n" <<
                 "- normal_k_search_: " << normal_k_search_ <<
                 "- visualize_: " << visualize_ << std::endl;

    if (config.readArray("association_modules"))
    {

        std::string type;
        config.nextArrayItem();
        if (config.value("type", type))
        {
            point_normal_alm_.configure(config);
            config.nextArrayItem();
        }

        if (config.value("type", type))
        {
            polygon_height_alm_.configure(config);
        }

        config.endArray();
    }

    if (config.readArray("segmentation_modules"))
    {
        std::string type;
        config.nextArrayItem();
        if (config.value("type", type))
        {
            euclidean_clustering_sm_.configure(config);

        }

        config.endArray();
    }

    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image

    rgbd::ImagePtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
    {
        ROS_WARN_STREAM("No RGBD image available for sensor '" << topic_ << "', is the RGBD Server running?");
        return;
    }


    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

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
    // Update sensor pose (localization)

//    std::set<ed::UUID> loc_ids;
//    loc_ids.insert(ed::UUID("plastic_cabinet"));

//    SamplingRenderLocalizer localizer;
//    SamplingProjectorLocalizer localizer;

//    tue::Timer timer;
//    timer.start();

//    sensor_pose = localizer.localize(sensor_pose, *rgbd_image, world, loc_ids);

//    std::cout << "Localization took " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Visualize

    if (visualize_)
    {
        rgbd::View view(*rgbd_image, 640);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat best_model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
        SampleRenderResult res(best_model);

        for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (e->shape())
            {
                geo::Pose3D pose = sensor_pose.inverse() * e->pose();
                geo::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), pose);

                // Render
                rasterizer.render(opt, res);
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

        cv::imshow("depth", rgbd_image->getDepthImage() / 8);
        cv::imshow("best model", best_model / 8);
        cv::imshow("diff", diff);
        cv::waitKey(3);
    }


    // - - - - - - - - - - - - - - - - - -
    // Create point cloud from rgbd data

    ed::RGBDData rgbd_data;
    rgbd_data.image = rgbd_image;
    rgbd_data.sensor_pose = sensor_pose;

    ed::helpers::ddp::extractPointCloud(rgbd_data, voxel_size_, max_range_, 1);
    ed::helpers::ddp::calculatePointCloudNormals(rgbd_data, normal_k_search_);

    if(visualize_)
    {
        ros::NodeHandle nh("~/kinect_plugin");
        ros::Publisher vis_marker_pub = nh.advertise<visualization_msgs::Marker>("vis_markers",0);
        ed::helpers::visualization::publishPclVisualizationMarker(sensor_pose,rgbd_data.point_cloud,vis_marker_pub, 0, "sensor_data_best_pose");
    }


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

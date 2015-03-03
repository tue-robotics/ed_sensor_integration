#include "plugin2.h"

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

KinectPlugin2::KinectPlugin2() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin2::~KinectPlugin2()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin2::configure(tue::Configuration config)
{
    std::string topic;
    if (config.value("topic", topic))
        kinect_client_.intialize(topic);

    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin2::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image

    rgbd::ImagePtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
    {
        ROS_WARN_STREAM("No RGBD image available");
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

    std::set<ed::UUID> loc_ids;
    loc_ids.insert(ed::UUID("plastic_cabinet"));

    SamplingRenderLocalizer localizer;
//    SamplingProjectorLocalizer localizer;

    tue::Timer timer;
    timer.start();

//    sensor_pose = localizer.localize(sensor_pose, *rgbd_image, world, loc_ids);

    std::cout << "Localization took " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // ...

    float asoc_dist = 0.1; // association distance at 1 m (the further away, the larger)

    const cv::Mat& depth_image = rgbd_image->getDepthImage();
    rgbd::View view_full(*rgbd_image, depth_image.cols);

    int asoc_dist_pixels = asoc_dist * view_full.getRasterizer().getFocalLengthX();

    rgbd::View view(*rgbd_image, depth_image.cols / asoc_dist_pixels);
    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat model(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
    SampleRenderResult res(model);

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

    cv::Mat model_min(model.rows, model.cols, CV_32FC1, 0.0);
    cv::Mat model_max(model.rows, model.cols, CV_32FC1, 0.0);

    for(int y = 0; y < model.rows; ++y)
    {
        for(int x = 0; x < model.cols; ++x)
        {
            float min = model.at<float>(y, x);
            float max = model.at<float>(y, x);
            for(int y2 = std::max(0, y - 1); y2 <= std::min(model.rows - 1, y + 1); ++y2)
            {
                for(int x2 = std::max(0, x - 01); x2 <= std::min(model.cols - 1, x + 1); ++x2)
                {
                    float d = model.at<float>(y2, x2);
                    if (d > 0)
                    {
                        min = std::min(min, d);
                        max = std::max(max, d);
                    }
                }
            }

            model_min.at<float>(y, x) = min;
            model_max.at<float>(y, x) = max;
        }
    }

    cv::Mat unassociated(depth_image.rows, depth_image.cols, CV_8UC3, cv::Scalar(0,0,0));

    float ratio = (float)model.cols / (float)depth_image.cols;

    for(int y = 0; y < depth_image.rows; ++y)
    {
        for(int x = 0; x < depth_image.cols; ++x)
        {
            float d = depth_image.at<float>(y, x);
            if (d == 0)
                continue;

            float d_min = model_min.at<float>(ratio * y, ratio * x) - d * asoc_dist;
            float d_max = model_max.at<float>(ratio * y, ratio * x) + d * asoc_dist;

            int c = (d / 8) * 255;

            if (d < d_min || d_max < d)
                unassociated.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, c);
            else
                unassociated.at<cv::Vec3b>(y, x) = cv::Vec3b(0, c, 0);
        }
    }

    cv::imshow("model", model / 8);
    cv::imshow("model min", model_min / 8);
    cv::imshow("model max", model_max / 8);
    cv::imshow("unassociated", unassociated);
    cv::waitKey(3);




    return;

    // - - - - - - - - - - - - - - - - - -
    // Visualize

    bool visualize = true;
    if (visualize)
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

}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin2)

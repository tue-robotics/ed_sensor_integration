#include "plugin.h"

#include <geolib/datatypes.h>
#include <geolib/ros/tf_conversions.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer_, cv::Mat& normal_map_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), normal_map(normal_map_), i_normal_offset(0),
          in_view(false) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
            normal_map.at<int>(y, x) = i_normal_offset + i_triangle;
            in_view = true;
        }
    }

    cv::Mat& z_buffer;
    cv::Mat& normal_map;
    int i_normal_offset;
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

void KinectPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    if (config.value("topic", topic_))
        kinect_client_.intialize(topic_);

    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image

    rgbd::ImagePtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
    {
        ROS_WARN_STREAM("No RGBD image available for sensor '" << topic_ << "'");
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

    std::cout << sensor_pose << std::endl;


    // - - - - - - - - - - - - - - - - - -
    // Convert depth map to point cloud

    const cv::Mat& depth = rgbd_image->getDepthImage();

    rgbd::View view(*rgbd_image, depth.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pc->width = depth.cols;
    pc->height = depth.rows;
    pc->is_dense = true; // may contain NaNs


    unsigned int size = depth.cols * depth.rows;
    pc->points.resize(size);

    unsigned int i = 0;
    for(int y = 0; y < depth.rows; ++y)
    {
        for(int x = 0; x < depth.cols; ++x)
        {
            float d = depth.at<float>(i);
            pcl::PointXYZ& p = pc->points[i];

            if (d > 0)
            {
                p.x = cam_model.project2Dto3DX(x) * d;
                p.y = cam_model.project2Dto3DY(y) * d;
                p.z = d;
            }
            else
            {
                p.x = NAN;
                p.y = NAN;
                p.z = NAN;
            }

            ++i;
        }
    }

    tue::Timer t_normal;
    t_normal.start();

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(20.0f);
    ne.setViewPoint(0, 0, 0);
    ne.setInputCloud(pc);
    ne.compute(*normals);

    std::cout << "Calculating normals took " << t_normal.getElapsedTimeInMilliSec() << " ms." << std::endl;

    // Visualize
    cv::Mat viz_normals(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for(unsigned int i = 0; i < size; ++i)
    {
        const pcl::Normal& n = normals->points[i];
        if (n.normal_x == n.normal_x)
        {
            int res = 255;

            int r = res * (n.normal_x + 1) / 2;
            int g = res * (n.normal_y + 1) / 2;
            int b = res * (n.normal_z + 1) / 2;

            r *= (255 / res);
            g *= (255 / res);
            b *= (255 / res);


            viz_normals.at<cv::Vec3b>(i) = cv::Vec3b(b, g, r);
        }
    }

    cv::imshow("normals", viz_normals);

    // - - - - - - - - - - - - - - - - - -
    // Render world model and calculate normals

    tue::Timer t_render;
    t_render.start();

    cv::Mat model(depth.rows, depth.cols, CV_32FC1, 0.0);
    cv::Mat normal_map(depth.rows, depth.cols, CV_32SC1, -1);
    std::vector<geo::Vector3> model_normals;

    SampleRenderResult res(model, normal_map);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape())
        {
            const geo::Mesh& mesh = e->shape()->getMesh();

            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(mesh, pose);

            // Render
            cam_model.render(opt, res);

            if (res.in_view)
            {
                const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
                const std::vector<geo::Vector3>& vertices = mesh.getPoints();

                for(unsigned int i = 0; i < triangles.size(); ++i)
                {
                    const geo::TriangleI& t = triangles[i];
                    const geo::Vector3& p1 = vertices[t.i1_];
                    const geo::Vector3& p2 = vertices[t.i2_];
                    const geo::Vector3& p3 = vertices[t.i3_];

                    // Calculate normal
                    geo::Vector3 n = ((p3 - p1).cross(p2 - p1)).normalized();

                    model_normals.push_back(sensor_pose.R * n);
                }

                res.i_normal_offset += triangles.size();
            }
        }
    }

    std::cout << "Rendering (with normals) took " << t_render.getElapsedTimeInMilliSec() << " ms." << std::endl;


    // Visualize
    cv::Mat viz_model_normals(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for(unsigned int i = 0; i < size; ++i)
    {
        int i_normal = normal_map.at<int>(i);

//        std::cout << i_normal << " / " << model_normals.size() << std::endl;


        if (i_normal >= 0)
        {
            const geo::Vector3& n = model_normals[i_normal];
            int res = 255;

            int r = res * (n.x + 1) / 2;
            int g = res * (n.y + 1) / 2;
            int b = res * (n.z + 1) / 2;

            r *= (255 / res);
            g *= (255 / res);
            b *= (255 / res);

            viz_model_normals.at<cv::Vec3b>(i) = cv::Vec3b(b, g, r);
        }
    }

    cv::imshow("model_normals", viz_model_normals);
    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

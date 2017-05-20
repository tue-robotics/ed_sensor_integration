#include "kinect_plugin.h"

#include <rgbd/Image.h>

#include <ros/node_handle.h>

#include <ed/uuid.h>
#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <tue/config/reader.h>

#include <rgbd/View.h>

#include "ed/kinect/association.h"

// GetImage
#include <rgbd/serialization.h>
#include <tue/serialization/conversions.h>
#include <ed/io/json_writer.h>
#include <ed/serialization/serialization.h>

#include <geolib/ros/msg_conversions.h>

//#include <opencv2/highgui/highgui.hpp>

#include "ray_tracer.h"

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;    

    std::string topic;
    if (config.value("topic", topic))
    {
        ROS_INFO_STREAM("[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic << "'.");
        image_buffer_.initialize(topic);
    }

    // - - - - - - - - - - - - - - - - - -
    // Services

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_get_image_ = nh.advertiseService("kinect/get_image", &KinectPlugin::srvGetImage, this);
    srv_update_ = nh.advertiseService("kinect/update", &KinectPlugin::srvUpdate, this);
    srv_ray_trace_ = nh.advertiseService("ray_trace", &KinectPlugin::srvRayTrace, this);

    ray_trace_visualization_publisher_ = nh.advertise<visualization_msgs::Marker>("ray_trace_visualization", 10);
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;   

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and pose

//    last_image_.reset();
//    if (!image_buffer_.nextImage("map", last_image_, last_sensor_pose_))
//        return;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();

    // - - - - - - - - - - - - - - - - - -

//    cv::imshow("kinect", last_image_->getRGBImage());
//    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvGetImage(ed_sensor_integration::GetImage::Request& req, ed_sensor_integration::GetImage::Response& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Get new image

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    if (!image_buffer_.waitForRecentImage("map", image, sensor_pose, 2.0))
    {
        res.error_msg = "Could not get image";
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Serialize RGBD image

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    rgbd::serialize(*image, a, rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);
    tue::serialization::convert(stream, res.rgbd_data);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write meta data

    std::stringstream meta_data;
    ed::io::JSONWriter w(meta_data);

    // Write sensor pose
    w.writeGroup("sensor_pose");
    ed::serialize(sensor_pose, w);
    w.endGroup();

    // Write rgbd filename
    w.writeValue("rgbd_filename", req.filename + ".rgbd");

    // Write timestamp
    w.writeGroup("timestamp");
    ed::serializeTimestamp(image->getTimestamp(), w);
    w.endGroup();

    w.finish();

    res.json_meta_data = meta_data.str();

    ROS_INFO_STREAM("[ED KINECT] Requested image. Image size: " << res.rgbd_data.size()
                    << " bytes. Meta-data size: " << res.json_meta_data.size() << " bytes.");

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvUpdate(ed_sensor_integration::Update::Request& req, ed_sensor_integration::Update::Response& res)
{    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Get new image

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    if (!image_buffer_.waitForRecentImage("map", image, sensor_pose, 2.0))
    {
        res.error_msg = "Could not get image";
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Perform update

    UpdateRequest kinect_update_req;
    kinect_update_req.area_description = req.area_description;
    kinect_update_req.background_padding = req.background_padding;

    // We expect the orientation of the supporting entity to be approximately correct.
    // Therefore, only allow rotation updates up to 45 degrees (both clock-wise and anti-clock-wise)
    kinect_update_req.max_yaw_change = 0.25 * M_PI;

    UpdateResult kinect_update_res(*update_req_);
    if (!updater_.update(*world_, image, sensor_pose, kinect_update_req, kinect_update_res))
    {
        res.error_msg = kinect_update_res.error.str();
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set result

    for(unsigned int i = 0; i < kinect_update_res.entity_updates.size(); ++i)
    {
        EntityUpdate& e_update = kinect_update_res.entity_updates[i];
        if (e_update.is_new)
            res.new_ids.push_back(e_update.id.str());
        else
            res.updated_ids.push_back(e_update.id.str());

        // Lock it, such that is won't be cleared by the clearer plugin
        update_req_->setFlag(e_update.id, "locked");
    }

    for(unsigned int i = 0; i < kinect_update_res.removed_entity_ids.size(); ++i)
        res.deleted_ids.push_back(kinect_update_res.removed_entity_ids[i].str());

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvRayTrace(ed_sensor_integration::RayTrace::Request& req, ed_sensor_integration::RayTrace::Response& res)
{
  if (req.raytrace_pose.header.frame_id != "/map" && req.raytrace_pose.header.frame_id != "map")
  {
    ROS_ERROR("KinectPlugin::srvRayTrace only works with poses expressed in /map frame");
    return false;
  }

  geo::Pose3D ray_trace_pose;
  geo::convert(req.raytrace_pose.pose, ray_trace_pose);

  ed_ray_tracer::RayTraceResult ray_trace_result = ed_ray_tracer::ray_trace(*world_, ray_trace_pose);

  if (!ray_trace_result.succes_)
  {
    ROS_ERROR("ed_ray_tracer::RayTrace failed!");
    return false;
  }

  res.entity_id = ray_trace_result.entity_id_;
  geo::convert(ray_trace_result.intersection_point_, res.intersection_point.point);
  res.intersection_point.header.stamp = ros::Time::now();
  res.intersection_point.header.frame_id = "map";

  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = "map";
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.color.a = 0.5;
  marker_msg.lifetime = ros::Duration(10.0);
  marker_msg.scale.x = 0.05;

  static int iter = 0;
  if ( ++iter % 2 == 0)
  {
    marker_msg.color.b = marker_msg.color.r = 1;
  }
  else
  {
    marker_msg.color.b = marker_msg.color.g = 1;
  }
  marker_msg.type = visualization_msgs::Marker::LINE_STRIP;

  marker_msg.points.push_back(req.raytrace_pose.pose.position);
  marker_msg.points.push_back(res.intersection_point.point);
  ray_trace_visualization_publisher_.publish(marker_msg);
  marker_msg.color.a = marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = 1;
  marker_msg.scale.x = 0.02;
  marker_msg.id = 1;
  ray_trace_visualization_publisher_.publish(marker_msg);

  return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

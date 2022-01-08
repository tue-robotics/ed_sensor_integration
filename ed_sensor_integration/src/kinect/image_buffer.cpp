#include "ed/kinect/image_buffer.h"

#include <rgbd/client.h>
#include <rgbd/image.h>
#include <tf/transform_listener.h>
#include <geolib/ros/tf_conversions.h>

#define MAX_BUFFER_TIME 1.0

// ----------------------------------------------------------------------------------------------------

ImageBuffer::ImageBuffer() : kinect_client_(nullptr), tf_listener_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

ImageBuffer::~ImageBuffer()
{
    if (kinect_client_)
        delete kinect_client_;
    if (tf_listener_)
        delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void ImageBuffer::initialize(const std::string& topic)
{
    if (!kinect_client_)
        kinect_client_ = new rgbd::Client;

    kinect_client_->initialize(topic);

    if (!tf_listener_)
        tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

bool ImageBuffer::waitForRecentImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec)
{
    if (!kinect_client_)
        return false;

    // - - - - - - - - - - - - - - - - - -
    // Wait until we get a new image

    ros::Time t_start = ros::Time::now();

    rgbd::ImageConstPtr rgbd_image;
    while(ros::ok())
    {
        if (ros::Time::now() - t_start > ros::Duration(timeout_sec))
            return false;

        rgbd_image = kinect_client_->nextImage();

        if (rgbd_image)
            break;
        else
            ros::Duration(0.1).sleep();
    }

    // - - - - - - - - - - - - - - - - - -
    // Wait until we have a tf

    if (!tf_listener_->waitForTransform(root_frame, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(timeout_sec)))
        return false;

    // - - - - - - - - - - - - - - - - - -
    // Calculate tf

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        return false;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    image = rgbd_image;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ImageBuffer::nextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose)
{
    if (!kinect_client_)
        return false;

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_->nextImage();
    if (rgbd_image)
        image_buffer_.push(rgbd_image);

    if (image_buffer_.empty())
        return false;

    rgbd_image = image_buffer_.front();

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
        image_buffer_.pop();

    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_DELAYED_THROTTLE(10, "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        ROS_WARN("[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());

        // Discard images that are out of date
        if (ros::Time::now() - ros::Time(rgbd_image->getTimestamp()) > ros::Duration(MAX_BUFFER_TIME))
            image_buffer_.pop();

        return false;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    image = rgbd_image;

    return true;
}

// ----------------------------------------------------------------------------------------------------



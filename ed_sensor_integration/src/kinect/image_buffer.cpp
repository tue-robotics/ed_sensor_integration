#include "ed/kinect/image_buffer.h"

#include <rgbd/client.h>
#include <rgbd/image.h>
#include <tf/transform_listener.h>
#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

ImageBuffer::ImageBuffer() : kinect_client_(nullptr), tf_listener_(nullptr), shutdown_(false)
{
}

// ----------------------------------------------------------------------------------------------------

ImageBuffer::~ImageBuffer()
{
    shutdown_ = true;
    if (worker_thread_ptr_)
        worker_thread_ptr_->join();
}

// ----------------------------------------------------------------------------------------------------

void ImageBuffer::initialize(const std::string& topic, const std::string& root_frame, const float worker_thread_frequency)
{
    root_frame_ = root_frame;

    if (!kinect_client_)
        kinect_client_ = std::make_unique<rgbd::Client>();

    kinect_client_->initialize(topic);

    if (!tf_listener_)
        tf_listener_ = std::make_unique<tf::TransformListener>();

    worker_thread_ptr_ = std::make_unique<std::thread>(&ImageBuffer::workerThreadFunc, this, worker_thread_frequency);
}

// ----------------------------------------------------------------------------------------------------

bool ImageBuffer::waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec)
{
    if (!kinect_client_)
    {
        ROS_ERROR("[IMAGE_BUFFER] No RGBD client");
        return false;
    }

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

    if (!tf_listener_->waitForTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(timeout_sec)))
        return false;

    // - - - - - - - - - - - - - - - - - -
    // Calculate tf

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
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

bool ImageBuffer::nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose)
{
    std::lock_guard<std::mutex> lg(recent_image_mutex_);
    if(!recent_image_.first)
    {
        ROS_DEBUG("[IMAGE_BUFFER] No new image");
        return false;
    }

    image = recent_image_.first;
    sensor_pose = recent_image_.second;

    recent_image_.first.reset(); // Invalidate the most recent image

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ImageBuffer::getMostRecentImageTF()
{
    if (!kinect_client_)
    {
        ROS_ERROR("[IMAGE_BUFFER] No RGBD client");
        return false;
    }

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_->nextImage();
    if (!rgbd_image)
    {
        ROS_DEBUG("[IMAGE_BUFFER] No new image from the RGBD client");
        return false;
    }

    geo::Pose3D sensor_pose;

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
            {
                ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "[IMAGE_BUFFER] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(rgbd_image->getTimestamp()));
                ROS_WARN_STREAM("[IMAGE_BUFFER] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(rgbd_image->getTimestamp()));
                return false;
            }
            else
            {
                if(!tf_listener_->waitForTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(0.5)))
                {
                    ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "[IMAGE_BUFFER] Image too new to look-up tf: image timestamp: " << ros::Time(rgbd_image->getTimestamp()) << ", what: " << ex.what());
                    ROS_WARN_STREAM("[IMAGE_BUFFER] Image too new to look-up tf: image timestamp: " << ros::Time(rgbd_image->getTimestamp()) << ", what: " << ex.what());
                    return false;
                }
                tf::StampedTransform t_sensor_pose;
                tf_listener_->lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
                geo::convert(t_sensor_pose, sensor_pose);
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_ERROR_DELAYED_THROTTLE(10, "[IMAGE_BUFFER] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            ROS_WARN("[IMAGE_BUFFER] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            return false;
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_DELAYED_THROTTLE(10, "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        ROS_WARN("[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
        return false;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    std::lock_guard<std::mutex> lg(recent_image_mutex_);
    recent_image_.first = rgbd_image;
    recent_image_.second = sensor_pose;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ImageBuffer::workerThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    while(!shutdown_)
    {
        getMostRecentImageTF();
        r.sleep();
    }
}

// ----------------------------------------------------------------------------------------------------



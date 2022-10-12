#include "ed/kinect/image_buffer.h"

#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/TransformStamped.h>

#include <rgbd/client.h>
#include <rgbd/image.h>

#include <tf2_ros/transform_listener.h>

// ----------------------------------------------------------------------------------------------------

ImageBuffer::ImageBuffer() : rgbd_client_(nullptr), tf_buffer_(), tf_listener_(nullptr), shutdown_(false)
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

    if (!rgbd_client_)
        rgbd_client_ = std::make_unique<rgbd::Client>();

    rgbd_client_->initialize(topic);

    if (!tf_listener_)
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    worker_thread_ptr_ = std::make_unique<std::thread>(&ImageBuffer::workerThreadFunc, this, worker_thread_frequency);
}

// ----------------------------------------------------------------------------------------------------

bool ImageBuffer::waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec)
{
    if (!rgbd_client_)
    {
        ROS_ERROR_NAMED("image_buffer", "[IMAGE_BUFFER] No RGBD client");
        return false;
    }

    // - - - - - - - - - - - - - - - - - -
    // Wait until we get a new image

    ros::Time t_start = ros::Time::now();
    ros::Time t_end = t_start + ros::Duration(timeout_sec);

    rgbd::ImageConstPtr rgbd_image;
    do
    {
        rgbd_image = rgbd_client_->nextImage();

        if (rgbd_image)
            break;
        else if (ros::Time::now() > t_end)
        {
            ROS_ERROR_NAMED("image_buffer", "[IMAGE_BUFFER] timeout waiting for rgbd image");
            return false;
        }
        else
            ros::Duration(0.1).sleep();
    }
    while(ros::ok()); // Give it minimal one go


    // - - - - - - - - - - - - - - - - - -
    // Wait until we have a tf
    if (!tf_buffer_.canTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()))) // Get the TF when it is available now
        if (!tf_buffer_.canTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_end - ros::Time::now()))
        {
            ROS_ERROR_NAMED("image_buffer", "[IMAGE_BUFFER] timeout waiting for tf");
            return false;
        }

    // - - - - - - - - - - - - - - - - - -
    // Calculate tf

    try
    {
        geometry_msgs::TransformStamped t_sensor_pose = tf_buffer_.lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()));
        geo::convert(t_sensor_pose.transform, sensor_pose);
    }
    catch(tf2::TransformException& ex)
    {
        ROS_ERROR_NAMED("image_buffer", "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
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
    if (!rgbd_client_)
    {
        ROS_ERROR("[IMAGE_BUFFER] No RGBD client");
        return false;
    }

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    {
        rgbd::ImageConstPtr new_image = rgbd_client_->nextImage();
        if (new_image)
        {
            image_buffer_.push_front(new_image);
            ROS_DEBUG_STREAM_NAMED("image_buffer", "[IMAGE_BUFFER] New image from the RGBD client with timestamp: " << std::fixed << std::setprecision(12) << new_image->getTimestamp());
        }
        else
        {
            ROS_DEBUG_NAMED("image_buffer", "[IMAGE_BUFFER] No new image from the RGBD client");
        }
    }

    geo::Pose3D sensor_pose;

    for (std::forward_list<rgbd::ImageConstPtr>::iterator it = image_buffer_.begin(); it != image_buffer_.end(); ++it)
    {
        rgbd::ImageConstPtr& rgbd_image = *it;
        try
        {
            geometry_msgs::TransformStamped t_sensor_pose = tf_buffer_.lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()));
            geo::convert(t_sensor_pose.transform, sensor_pose);
        }
        catch (tf2::ExtrapolationException& ex)
        {
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
                // to new, respectively). If it is too old, discard it.
                geometry_msgs::TransformStamped latest_sensor_pose = tf_buffer_.lookupTransform(root_frame_, rgbd_image->getFrameId(), ros::Time(0));
                // If image time stamp is older than latest transform, the image is too old and the tf data is not available anymore
                if ( latest_sensor_pose.header.stamp > ros::Time(rgbd_image->getTimestamp()) )
                {

                    ROS_DEBUG_STREAM_NAMED("image_buffer", "[IMAGE_BUFFER] Image too old to look-up tf. Deleting all images older than timestamp: " << std::fixed
                                           << ros::Time(rgbd_image->getTimestamp()));
                    // Deleting this image and all older images
                    image_buffer_.erase_after(it, image_buffer_.end());
                    return false;
                }
                else
                {
                    // Image is too new; continue to next image, which is older
                    ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(10, "image_buffer", "[IMAGE_BUFFER] Image too new to look-up tf: image timestamp: " << std::fixed << ros::Time(rgbd_image->getTimestamp()) << ", what: " << ex.what());
                    continue;
                }
            }
            catch (tf2::TransformException& ex)
            {
                ROS_ERROR_DELAYED_THROTTLE_NAMED(10, "image_buffer", "[IMAGE_BUFFER] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
                ROS_DEBUG_NAMED("image_buffer", "[IMAGE_BUFFER] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
                continue;
            }
        }
        catch (tf2::TransformException& ex)
        {
            ROS_ERROR_DELAYED_THROTTLE_NAMED(10, "image_buffer", "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
            ROS_WARN_NAMED("image_buffer", "[IMAGE_BUFFER] Could not get sensor pose: %s", ex.what());
            continue;
        }

        // Convert from ROS coordinate frame to geolib coordinate frame
        sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

        {
            std::lock_guard<std::mutex> lg(recent_image_mutex_);
            recent_image_.first = rgbd_image;
            recent_image_.second = sensor_pose;
        }

        // Deleting this image and all older images
        image_buffer_.erase_after(it, image_buffer_.end());

        return true;
    }

    // Not been able to update the most recent image/TF combo
    return false;
}

// ----------------------------------------------------------------------------------------------------

void ImageBuffer::workerThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    while(!shutdown_)
    {
        if (!getMostRecentImageTF())
            ROS_ERROR_DELAYED_THROTTLE_NAMED(5, "image_buffer", "[IMAGE_BUFFER] Could not get a pose for any image in the buffer");
        r.sleep();
    }
}

// ----------------------------------------------------------------------------------------------------



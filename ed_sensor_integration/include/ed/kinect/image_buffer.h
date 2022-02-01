#ifndef ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_
#define ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_

#include <queue>
#include <ros/callback_queue.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>

namespace rgbd
{
class Client;
}

namespace tf
{
class TransformListener;
}

/**
 * @brief The ImageBuffer class, provides a buffer to synchronise rgbd images with sensor positions.
 * Images are stored until they are removed from the buffer or until a
 * certain amount of time has passed
 */
class ImageBuffer
{

public:

    ImageBuffer();

    ~ImageBuffer();

    /**
     * @brief initialize, configure the buffer
     * @param topic, ros topic on which the rgbd images are published
     */
    void initialize(const std::string& topic);

    /**
     * @brief nextImage, Returns the most recent combination of image and transform, provided it is different from the previous time the function was called.
     *
     * Will remove the found image from the buffer.
     *
     * @param[in] root_frame, name of the tf frame with respect to which the sensor pose should be expressed
     * @param[out] image, rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose, will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @return whether or not a novel image is available
     */
    bool nextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

    //
    /**
     * @brief waitForRecentImage, Blocks until a new image with transform is found. Returns false if no image or tf could be found within 'timeout_sec' seconds
     *
     * Will remove the found image from the buffer.
     *
     * @param[out] root_frame, name of the tf frame with respect to which the sensor pose should be expressed
     * @param[out] image, rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose, will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @param[in] timeout_sec, maximum duration to block.
     * @return whether or not a next image was available within the timeout duration.
     */
    bool waitForRecentImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec);

private:

    std::string topic_;

    rgbd::Client* kinect_client_;

    tf::TransformListener* tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;


};

#endif

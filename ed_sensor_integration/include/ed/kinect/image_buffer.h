#ifndef ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_
#define ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_

#include <ros/callback_queue.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>

#include <memory>
#include <mutex>
#include <queue>
#include <thread>

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
     * @param root_frame, root frame to calculate sensor pose relative to (default: map)
     * @param worker_thread_frequency, frequency at which the worker thread updates the most recent image (default: 20)
     */
    void initialize(const std::string& topic, const std::string& root_frame="map", const float worker_thread_frequency=20);

    /**
     * @brief nextImage, Returns the most recent combination of image and transform, provided it is different from the previous time the function was called.
     *
     * @param[out] image, rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose, will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @return whether or not a novel image is available
     */
    bool nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

    /**
     * @brief waitForRecentImage, Blocks until a new image with transform is found. Returns false if no image or tf could be found within 'timeout_sec' seconds
     *
     * @param[out] image, rgbd image to write the next image to. Iff a next image is found
     * @param[out] sensor_pose, will be filled with the sensor pose corresponding to the next image. Iff a next image is found
     * @param[in] timeout_sec, maximum duration to block.
     * @return whether or not a next image was available within the timeout duration.
     */
    bool waitForRecentImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec);

private:

    std::string root_frame_;

    std::unique_ptr<rgbd::Client> kinect_client_;

    std::unique_ptr<tf::TransformListener> tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;

    std::pair<rgbd::ImageConstPtr, geo::Pose3D> recent_image_;
    std::mutex recent_image_mutex_;
    std::unique_ptr<std::thread> worker_thread_ptr_;
    bool shutdown_;

    bool getMostRecentImageTF();

    void workerThreadFunc(const float frequency=20);

};

#endif

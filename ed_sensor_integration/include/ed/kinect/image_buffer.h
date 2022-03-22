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

class ImageBuffer
{

public:

    ImageBuffer();

    ~ImageBuffer();

    void initialize(const std::string& topic, const std::string& root_frame="map", const float worker_thread_frequency=20);

    // Polls to see if there is a new image with transform. If not, returns false
    bool nextImage(rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

    // Blocks until a new image with transform is found. Returns false if no image or tf could be found within 'timeout_sec' seconds
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

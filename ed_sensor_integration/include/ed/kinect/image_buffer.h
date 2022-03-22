#ifndef ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_
#define ED_SENSOR_INTEGRATION_IMAGE_BUFFER_H_

#include <ros/callback_queue.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>

#include <memory>
#include <queue>

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

    void initialize(const std::string& topic);

    // Polls to see if there is a new image with transform. If not, returns false
    bool nextImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose);

    // Blocks until a new image with transform is found. Returns false if no image or tf could be found within 'timeout_sec' seconds
    bool waitForRecentImage(const std::string& root_frame, rgbd::ImageConstPtr& image, geo::Pose3D& sensor_pose, double timeout_sec);

private:

    std::unique_ptr<rgbd::Client> kinect_client_;

    std::unique_ptr<tf::TransformListener> tf_listener_;

    std::queue<rgbd::ImageConstPtr> image_buffer_;

    ros::CallbackQueue cb_queue_;


};

#endif

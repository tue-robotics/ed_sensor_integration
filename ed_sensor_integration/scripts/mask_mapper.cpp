#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class ImageSubscriber {
public:
    ImageSubscriber() : it_(nh_) {
        //subscribe to image and mask topics
        mask_sub_ = it_.subscribe("/hero/segmented_table_mask", 1, &ImageSubscriber::maskCallback, this); 
        original_image_sub_ = it_.subscribe("/hero/head_rgbd_sensor/rgb/image_raw", 1, &ImageSubscriber::imageCallback, this);
        cv::namedWindow("Overlay", cv::WINDOW_NORMAL);
    }

    void maskCallback(const sensor_msgs::ImageConstPtr& mask_msg) {
        try {
            //convert mask msg to opencv image
            cv::Mat mask = cv_bridge::toCvShare(mask_msg, "mono8")->image;
            updateOverlay(mask);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
        try {
            //convert image msg to opencv image
            cv::Mat image = cv_bridge::toCvShare(image_msg, "passthrough")->image;
            overlayMask(image);
            cv::imshow("Overlay", image);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void overlayMask(cv::Mat& image) {
        //maps mask onto image
        cv::bitwise_and(image, image, image, mask);
    }

    void updateOverlay(const cv::Mat& mask) {

        overlayMask(original_image_);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber mask_sub_;
    image_transport::Subscriber original_image_sub_;
    cv::Mat original_image_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber");
    ImageSubscriber image_subscriber;
    ros::spin();
    return 0;
}

#include "map_filter.h"

#include <ros/node_handle.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

MapFilter::MapFilter() : send_update_(false) {}

// ----------------------------------------------------------------------------------------------------

void MapFilter::initialize(const std::string& in_topic, const std::string& out_topic)
{
    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    pub_ = nh.advertise<nav_msgs::OccupancyGrid>(out_topic, 1, true); // latching
    sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(in_topic, 1, &MapFilter::mapCallback, this);
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::setEntityPose(const geo::Transform2& pose, const std::vector<std::vector<geo::Vec2> >& contours)
{
    if (!mask_.data)
        return;

//    std::vector<std::vector<cv::Point> > cv_contours;

//    for(std::vector<std::vector<geo::Vec2> >::const_iterator it = contours.begin(); it != contours.end(); ++it)
//    {
//        const std::vector<geo::Vec2>& contour = *it;
//        std::vector<cv::Point> points(contour.size());
//        for(unsigned int i = 0; i < contour.size(); ++i)
//        {
//            geo::Vec2 p_MAP = pose * contour[i];
//            points[i] = cv::Point((p_MAP.x - map_origin_.x) / res_, (p_MAP.y - map_origin_.y) / res_);
//        }

//        cv_contours.push_back(points);
//    }

//    cv::fillPoly(mask_, cv_contours, cv::Scalar(0));

    send_update_ = true;
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::update()
{
//    return;

    map_in_.reset();
    cb_queue_.callAvailable();

    if (!map_in_ && !send_update_)
        return;

    send_update_ = false;

    if (map_in_)
        std::cout << "[ED FITTER] Received new map or had an updated entity pose!" << std::endl;

    unsigned int w = map_in_->info.width;
    unsigned int h = map_in_->info.height;
    res_ = map_in_->info.resolution;

    map_origin_.x = map_in_->info.origin.position.x;
    map_origin_.y = map_in_->info.origin.position.y;

    if (!mask_.data)
    {
        mask_ = cv::Mat(h, w, CV_8SC1, cv::Scalar(255));
    }
    else
    {

    }

    cv::imshow("MASK", mask_);
    cv::waitKey(3);

//    nav_msgs::OccupancyGrid map_out = *map_in_;

//    for(unsigned int i = 0; i < w * h; ++i)
//    {
//        if (mask_.at<char>(i) == 0)
//            map_out.data[i] = 100;
//    }

//    pub_.publish(map_out);
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_in_ = msg;
}

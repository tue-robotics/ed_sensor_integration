#include "map_filter.h"

#include <ros/node_handle.h>

#include <opencv2/highgui/highgui.hpp>

#include <geolib/HeightMap.h>
#include <geolib/CompositeShape.h>

#include <opencv2/imgproc/imgproc.hpp>

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

void MapFilter::worldToGrid(double x, double y, int& mx, int& my)
{
    mx = (x - map_origin_.x) / res_;
    my = (y - map_origin_.y) / res_;
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::gridToWorld(int mx, int my, double& x, double& y)
{
    x = mx * res_ + map_origin_.x;
    y = my * res_ + map_origin_.y;
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::setEntityPose(const geo::Transform2& pose, const std::vector<std::vector<geo::Vec2> >& contours,
                              double obstacle_inflation)
{
    std::cout << "MapFilter::setEntityPose" << std::endl;

    if (!mask_.data)
        return;

    std::vector<std::vector<cv::Point> > cv_contours;

    for(std::vector<std::vector<geo::Vec2> >::const_iterator it = contours.begin(); it != contours.end(); ++it)
    {
        const std::vector<geo::Vec2>& contour = *it;
        std::vector<cv::Point> points(contour.size());
        for(unsigned int i = 0; i < contour.size(); ++i)
        {
            geo::Vec2 p_MAP = pose * contour[i];

            int mx, my;
            worldToGrid(p_MAP.x, p_MAP.y, mx, my);

            points[i] = cv::Point(mx, my);
        }

        cv_contours.push_back(points);

        for(unsigned int i = 0; i < contour.size(); ++i)
        {
            int j = (i + 1) % contour.size();
            cv::line(mask_, points[i], points[j], cv::Scalar(0), obstacle_inflation / res_ + 1);
        }
    }

    cv::fillPoly(mask_, cv_contours, cv::Scalar(0));

    send_update_ = true;
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::update()
{
//    if (mask_.data)
//    {
//        cv::Mat img;
//        cv::resize(mask_, img, cv::Size(mask_.cols / 3, mask_.rows / 3));

//        cv::imshow("MASK", img);
//        cv::waitKey(3);
//    }

    cb_queue_.callAvailable();

    if (!map_in_ || !send_update_)
        return;

    send_update_ = false;

    std::cout << "[ED FITTER] Received new map or had an updated entity pose!" << std::endl;

    unsigned int w = map_in_->info.width;
    unsigned int h = map_in_->info.height;

    res_ = map_in_->info.resolution;

    if (!mask_.data)
    {
        map_origin_.x = map_in_->info.origin.position.x;
        map_origin_.y = map_in_->info.origin.position.y;

        mask_ = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));
    }
    else
    {
        if (mask_.cols != w || mask_.rows != h)
        {
            // Resize!

            int dx = (map_origin_.x - map_in_->info.origin.position.x) / res_;
            int dy = (map_origin_.y - map_in_->info.origin.position.y) / res_;

            std::cout << "dx = " << dx << ", dy = " << dy << std::endl;

            int w_new = std::max<int>(dx + mask_.cols, w) + 1;
            int h_new = std::max<int>(dy + mask_.rows, h) + 1;

            cv::Mat new_mask = cv::Mat(h_new, w_new, CV_8UC1, cv::Scalar(255));

            for(int y = 0; y < mask_.rows; ++y)
            {
                for(int x = 0; x < mask_.cols; ++x)
                {
                    new_mask.at<unsigned char>(y + dy, x + dx) = mask_.at<unsigned char>(y, x);
                }
            }

            std::cout << "old mask size: " << mask_.cols << " x " << mask_.rows << std::endl;
            std::cout << "new mask size: " << new_mask.cols << " x " << new_mask.rows << std::endl;

//            cv::Mat roi = new_mask(cv::Rect(cv::Point(dx, dy), cv::Point(mask_.cols, mask_.rows)));
//            mask_.copyTo(roi);

//            cv::Mat tmp_mask = new_mask.clone();

            map_origin_.x = map_in_->info.origin.position.x;
            map_origin_.y = map_in_->info.origin.position.y;

            mask_ = new_mask;

            std::cout << "Done!" << std::endl;
        }
    }

    nav_msgs::OccupancyGrid msg;
    filtered_map_ = *map_in_;

    int k = 0;
    for(unsigned int y = 0; y < h; ++y)
    {
        for(unsigned int x = 0; x < w; ++x)
        {
            if (mask_.at<unsigned char>(y, x) == 0)
                filtered_map_.data[k] = 0;

            ++k;
        }
    }

    pub_.publish(filtered_map_);
}

// ----------------------------------------------------------------------------------------------------

geo::ShapeConstPtr MapFilter::createWallShape(double height)
{
    // Make sure we have the latest map
    update();

    if (filtered_map_.data.empty())
        return geo::ShapeConstPtr();

    int w = filtered_map_.info.width;
    int h = filtered_map_.info.height;

    // ------------------------------------------------------------------------------------------

    int k = 0;
    std::vector<std::vector<double> > grid(w, std::vector<double>(h, 0));
    for(unsigned int y = 0; y < h; ++y)
    {
        for(unsigned int x = 0; x < w; ++x)
        {
            if (filtered_map_.data[k] > 0)
                grid[x][y] = height;

            ++k;
        }
    }

    geo::Pose3D pose(geo::Mat3::identity(), geo::Vec3(map_origin_.x, map_origin_.y, 0));

    geo::Mesh mesh = geo::HeightMap::fromGrid(grid, res_).getMesh();

    geo::ShapePtr shape(new geo::Shape);
    shape->setMesh(mesh.getTransformed(pose));

    // ------------------------------------------------------------------------------------------

    std::cout << "Walls have: " << shape->getMesh().getTriangleIs().size() << " triangles" << std::endl;

    return shape;
}

// ----------------------------------------------------------------------------------------------------

void MapFilter::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_in_ = msg;
    send_update_ = true;
}

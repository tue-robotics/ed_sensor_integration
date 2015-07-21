#include "map_filter.h"

#include <ros/node_handle.h>

#include <opencv2/highgui/highgui.hpp>

#include <geolib/HeightMap.h>
#include <geolib/CompositeShape.h>

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

geo::ShapeConstPtr MapFilter::createWallShape(double height)
{
    // Make sure we have the latest map
    cb_queue_.callAvailable();

    if (!map_in_)
        return geo::ShapeConstPtr();

    int w = map_in_->info.width;
    int h = map_in_->info.height;
    res_ = map_in_->info.resolution;

    map_origin_.x = map_in_->info.origin.position.x;
    map_origin_.y = map_in_->info.origin.position.y;

    // ------------------------------------------------------------------------------------------

    //    geo::CompositeShape comp_shape;

    //    int k = 0;
    ////    std::vector<std::vector<double> > grid(w, std::vector<double>(h, 0));
    //    for(unsigned int y = 0; y < h; ++y)
    //    {
    //        for(unsigned int x = 0; x < w; ++x)
    //        {
    //            if (map_in_->data[k] > 0)
    //            {
    //                double x1 = res_ * x + map_origin_.x;
    //                double x2 = res_ * (x + 1) + map_origin_.x;
    //                double y1 = res_ * y + map_origin_.y;
    //                double y2 = res_ * (y + 1) + map_origin_.y;

    //                comp_shape.addShape(geo::Box(geo::Vec3(x1, y1, 0), geo::Vec3(x2, y2, height)), geo::Pose3D::identity());
    //            }
    //            ++k;
    //        }
    //    }

    //    geo::Mesh mesh = geo::HeightMap::fromGrid(grid, res_).getMesh();
    //    geo::ShapePtr shape(new geo::Shape);
    //    shape->setMesh(mesh);

    //    const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

    //    for(unsigned int i= 0;i < vertices.size(); ++i)
    //    {
    //        std::cout << vertices[i] << std::endl;
    //    }

    //    geo::serialization::toFile(geo::HeightMap::fromGrid(grid, res_), "/tmp/heightmap.geo");

//    geo::ShapeConstPtr shape(new geo::Shape(comp_shape));

    //    geo::serialization::toFile(*shape, "/tmp/walls.geo");



    // ------------------------------------------------------------------------------------------

    int k = 0;
    std::vector<std::vector<double> > grid(w, std::vector<double>(h, 0));
    for(unsigned int y = 0; y < h; ++y)
    {
        for(unsigned int x = 0; x < w; ++x)
        {
            if (map_in_->data[k] > 0)
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
}

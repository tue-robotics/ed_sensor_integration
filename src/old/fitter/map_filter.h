#ifndef ED_SENSOR_INTEGRATION_FITTER_MAP_FILTER_H_
#define ED_SENSOR_INTEGRATION_FITTER_MAP_FILTER_H_

#include <geolib/datatypes.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/callback_queue.h>

#include <opencv2/core/core.hpp>

#include <nav_msgs/OccupancyGrid.h>

class MapFilter
{

public:

    MapFilter();

    void initialize(const std::string& in_topic, const std::string& out_topic);

    void setEntityPose(const geo::Transform2& pose, const std::vector<std::vector<geo::Vec2> >& contour, double obstacle_inflation);

    geo::ShapeConstPtr createWallShape(double height);

    void update();

private:

    cv::Mat mask_;

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_;

    ros::Publisher pub_;

    double res_;

    geo::Vec2 map_origin_;

    bool send_update_;

    nav_msgs::OccupancyGrid::ConstPtr map_in_;

    nav_msgs::OccupancyGrid filtered_map_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void worldToGrid(double x, double y, int& mx, int& my);

    void gridToWorld(int mx, int my, double& x, double& y);


};

#endif

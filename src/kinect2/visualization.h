#ifndef ED_SENSOR_INTEGRATION_KINECT2_VISUALIZATION_H_
#define ED_SENSOR_INTEGRATION_KINECT2_VISUALIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ed/helpers/image_publisher.h>
#include <ed/types.h>
#include <geolib/datatypes.h>

#include <rgbd/View.h>

void visualizeNormals(const pcl::PointCloud<pcl::PointNormal>& pc, ed::ImagePublisher& pub);

void visualizeClusters(const cv::Mat& rgb, const std::vector<std::vector<unsigned int> >& clusters, ed::ImagePublisher& pub);

//void visualizeWorldModel(const ed::WorldModel& world, const geo::Pose3D& sensor_pose, const rgbd::View& view, ed::ImagePublisher& pub);

#endif

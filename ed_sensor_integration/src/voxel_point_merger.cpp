#include "ed_sensor_integration/voxel_point_merger.h"

#include <pcl/filters/voxel_grid.h>
#include <ros/console.h>

namespace ed_sensor_integration
{

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelPointMerger::merge(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& old_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud)
{
    // Handle first observation case
    if (!old_cloud || old_cloud->empty())
    {
        if (!new_cloud || new_cloud->empty())
        {
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        }

        // Apply voxel filter to new cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(new_cloud);
        voxel_filter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
        voxel_filter.filter(*filtered_cloud);

        ROS_DEBUG("VoxelPointMerger: First observation - %zu points → %zu voxels",
                  new_cloud->size(), filtered_cloud->size());

        return filtered_cloud;
    }

    // Combine old and new clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *combined_cloud = *old_cloud;
    *combined_cloud += *new_cloud;

    // Apply voxel grid filter to downsample the combined cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(combined_cloud);
    voxel_filter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    voxel_filter.filter(*merged_cloud);

    ROS_DEBUG("VoxelPointMerger: Merged %zu old + %zu new → %zu voxels",
              old_cloud->size(), new_cloud->size(), merged_cloud->size());

    return merged_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelPointMerger::convertToPCL(
    const std::vector<geo::Vec3>& points,
    const geo::Pose3D& sensor_pose)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.reserve(points.size());

    for (const geo::Vec3& p_sensor : points)
    {
        // Transform from sensor frame to map frame
        geo::Vec3 p_map = sensor_pose * p_sensor;

        pcl::PointXYZ pcl_point;
        pcl_point.x = p_map.x;
        pcl_point.y = p_map.y;
        pcl_point.z = p_map.z;
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

std::vector<geo::Vec3> VoxelPointMerger::convertToGeo(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<geo::Vec3> points;
    if (!cloud)
        return points;

    points.reserve(cloud->size());

    for (const pcl::PointXYZ& pcl_point : cloud->points)
    {
        points.emplace_back(pcl_point.x, pcl_point.y, pcl_point.z);
    }

    return points;
}

} // namespace ed_sensor_integration

#ifndef ED_SENSOR_INTEGRATION_VOXEL_POINT_MERGER_H_
#define ED_SENSOR_INTEGRATION_VOXEL_POINT_MERGER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geolib/datatypes.h>
#include <vector>

namespace ed_sensor_integration
{

/**
 * @brief Voxel-based point cloud merger using PCL VoxelGrid filter
 *
 * Merges multiple point cloud observations of the same object using voxel downsampling
 * to maintain bounded memory while preserving shape fidelity. This is the industry-standard
 * approach used in robotics perception (costmap_2d, OctoMap, MoveIt).
 */
class VoxelPointMerger
{
public:
    /**
     * @brief Voxel leaf size for downsampling (1cm default for tabletop objects)
     *
     * Smaller values: more detail, higher memory/compute
     * Larger values: faster processing, less detail
     */
    static constexpr float VOXEL_LEAF_SIZE = 0.01f; // 1 cm

    /**
     * @brief Merge old and new point clouds using voxel downsampling
     *
     * @param old_cloud Previous accumulated point cloud in map frame (can be nullptr for first observation)
     * @param new_cloud New measurement point cloud in map frame
     * @return Merged and voxel-downsampled point cloud
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr merge(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& old_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud);

    /**
     * @brief Convert geo::Vec3 points to PCL point cloud in map frame
     *
     * @param points Points in sensor frame
     * @param sensor_pose Sensor pose (sensor → map transform)
     * @return PCL point cloud in map frame
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(
        const std::vector<geo::Vec3>& points,
        const geo::Pose3D& sensor_pose);

    /**
     * @brief Convert PCL point cloud back to geo::Vec3 vector
     *
     * @param cloud PCL point cloud
     * @return Vector of geo::Vec3 points
     */
    static std::vector<geo::Vec3> convertToGeo(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

} // namespace ed_sensor_integration

#endif // ED_SENSOR_INTEGRATION_VOXEL_POINT_MERGER_H_

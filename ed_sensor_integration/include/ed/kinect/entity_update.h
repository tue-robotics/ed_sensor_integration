#ifndef ED_KINECT_ENTITY_UPDATE_H_
#define ED_KINECT_ENTITY_UPDATE_H_

#include <ed/uuid.h>
#include <ed/convex_hull.h>
#include <vector>
#include <geolib/datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct EntityUpdate
{
    // Association
    bool is_new;
    ed::UUID id;

    // Measurement
    std::vector<unsigned int> pixel_indices;
    std::vector<geo::Vec3> points;

    // Shape
    ed::ConvexHull chull;

    // Pose
    geo::Pose3D pose_map;

    // Optional: outliers from filtering (e.g., GMM) -- Leave empty if not applicable
    std::vector<geo::Vec3> outlier_points;

    // Optional: accumulated point cloud in map frame for voxel-based merging
    // This is transient (not persisted to ED) and used for multi-observation tracking
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_map;
};

#endif

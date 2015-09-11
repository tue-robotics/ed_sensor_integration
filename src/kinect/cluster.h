#ifndef ED_SENSOR_INTEGRATION_KINECT_CLUSTER_H_
#define ED_SENSOR_INTEGRATION_KINECT_CLUSTER_H_

#include <geolib/datatypes.h>
#include <ed/convex_hull.h>

struct Cluster
{
    std::vector<unsigned int> pixel_indices;
    std::vector<geo::Vec3> points;
    ed::ConvexHull chull;
    geo::Pose3D pose_map;
};

#endif

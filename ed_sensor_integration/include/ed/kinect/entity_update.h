#ifndef ED_KINECT_ENTITY_UPDATE_H_
#define ED_KINECT_ENTITY_UPDATE_H_

#include <ed/uuid.h>
#include <ed/convex_hull.h>
#include <vector>
#include <string>
#include <geolib/datatypes.h>

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

    // Classification (from YOLO) -- Empty label means no classification available
    std::string label;
    float classification_confidence = 0.0f;

    // Optional: outliers from filtering (e.g., GMM) -- Leave empty if not applicable
    std::vector<geo::Vec3> outlier_points;
};

#endif

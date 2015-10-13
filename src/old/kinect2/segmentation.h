#ifndef ED_ROBOCUP_SEGMENTATION_H_
#define ED_ROBOCUP_SEGMENTATION_H_

#include <ed/types.h>
#include <ed/convex_hull.h>
#include <ed/mask.h>

#include <rgbd/types.h>
#include <geolib/datatypes.h>

struct Cluster
{
    std::vector<unsigned int> pixels;
    ed::ImageMask image_mask;
    ed::ConvexHull chull;
    geo::Pose3D pose;
};

struct Segmenter
{

    Segmenter() : association_correspondence_distance_(0.3), downsample_factor_(1), max_range_(3)
    {
    }

    void segment(const ed::WorldModel& world, const rgbd::Image& image, const geo::Pose3D& sensor_pose, std::vector<Cluster>& segments);

    float association_correspondence_distance_;

    int downsample_factor_;

    float max_range_;

};

#endif

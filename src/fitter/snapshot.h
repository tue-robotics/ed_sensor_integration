#ifndef ED_SENSOR_INTEGRATION_FITTER_SNAPSHOT_H_
#define ED_SENSOR_INTEGRATION_FITTER_SNAPSHOT_H_

#include <rgbd/Image.h>
#include <ed/uuid.h>

struct Snapshot
{
    Snapshot() : revision(0) {}

    rgbd::ImageConstPtr image;      // original camera image
    double first_timestamp;         // timestamp of first image (not current image!)
    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    cv::Mat background_image;       // background image (from original camera image)
    cv::Mat canvas;                 // camera image including visualizations
    std::set<ed::UUID> visualized_ids;
    unsigned int revision;
};

#endif

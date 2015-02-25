#ifndef ED_KINECT_SAMPLING_PROJECTOR_LOCALIZER_H_
#define ED_KINECT_SAMPLING_PROJECTOR_LOCALIZER_H_

#include <geolib/datatypes.h>
#include <rgbd/types.h>
#include <ed/types.h>

class SamplingProjectorLocalizer
{

public:

    SamplingProjectorLocalizer();

    ~SamplingProjectorLocalizer();

    geo::Pose3D localize(const geo::Pose3D& sensor_pose, const rgbd::Image& image, const ed::WorldModel& world);

};

#endif

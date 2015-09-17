#ifndef _RENDERER_H_
#define _RENDERER_H_

#include <ed/types.h>
#include <geolib/datatypes.h>
#include <geolib/sensors/DepthCamera.h>
#include <rgbd/types.h>

// Samples sensor pose in Z (height), Roll and Pitch. Uses sensor_pose as seed. The sampled sensor_pose at
// which the shape fits the given RGBD image best is returned.
void fitZRP(const geo::Shape& shape, const geo::Pose3D& shape_pose, const rgbd::Image& image,
            const geo::Pose3D& sensor_pose, geo::Pose3D& updated_sensor_pose);


#endif

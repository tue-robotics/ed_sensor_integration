#ifndef ED_SENSOR_INTEGRATION_FITTER_GUI_H_
#define ED_SENSOR_INTEGRATION_FITTER_GUI_H_

#include <rgbd/types.h>
#include <ed/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

void DrawWorldModelOverlay(const rgbd::Image& image, const geo::Pose3D& sensor_pose, const ed::WorldModel& world,
                           const std::set<ed::UUID>& ids, cv::Mat& canvas, bool& changed);

#endif

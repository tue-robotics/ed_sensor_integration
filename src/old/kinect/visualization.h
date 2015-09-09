#ifndef ED_SENSOR_INTEGRATION_KINECT_VISUALIZATION_H_
#define ED_SENSOR_INTEGRATION_KINECT_VISUALIZATION_H_

#include <ed/types.h>
#include <rgbd/types.h>
#include <opencv2/core/core.hpp>

// ----------------------------------------------------------------------------------------------------

void drawImageWithMeasurements(const ed::WorldModel& world_model, rgbd::ImageConstPtr rgbd_image, cv::Mat& color_img);

#endif

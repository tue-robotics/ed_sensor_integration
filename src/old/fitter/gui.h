#ifndef ED_SENSOR_INTEGRATION_FITTER_GUI_H_
#define ED_SENSOR_INTEGRATION_FITTER_GUI_H_

#include <rgbd/types.h>
#include <ed/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

#include <ros/publisher.h>

#include "snapshot.h"

#include <cb_planner_msgs_srvs/LocalPlannerAction.h>

void DrawWorldModelOverlay(const ed::WorldModel& world, const std::set<ed::UUID>& ids,
                           const std::set<ed::UUID>& updated_ids, Snapshot& snapshot,
                           bool force_change, bool& changed);

class Navigator
{

public:

    void initialize(ros::NodeHandle& nh, const std::string& goal_topic);

    bool navigate(const Snapshot& snapshot, int click_x, int click_y);

private:

    ros::Publisher pub_goal_;

};

#endif

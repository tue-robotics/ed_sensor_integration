#ifndef ED_SENSOR_INTEGRATION_FITTER_GUI_H_
#define ED_SENSOR_INTEGRATION_FITTER_GUI_H_

#include <rgbd/types.h>
#include <ed/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

#include "snapshot.h"

void DrawWorldModelOverlay(const ed::WorldModel& world, const std::set<ed::UUID>& ids,
                           const std::set<ed::UUID>& updated_ids, Snapshot& snapshot, bool& changed);

#endif

#ifndef ED_RAY_TRACER_H_
#define ED_ROBOCUP_VISUALIZER_H_

#include <ed/types.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>
#include <set>

namespace ed_ray_tracer
{

struct RayTraceResult {
  RayTraceResult() :
    succes_(false)
  {

  }

  bool succes_;
  std::string entity_id_;
  geo::Vector3 intersection_point_;
};

RayTraceResult ray_trace(const ed::WorldModel& world, const geo::Pose3D& raytrace_pose);

}

#endif

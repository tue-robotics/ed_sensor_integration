#include "ray_tracer.h"

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <geolib/sensors/LaserRangeFinder.h>

#include <ros/console.h>

namespace ed_ray_tracer
{

// ----------------------------------------------------------------------------------------------------

class PointRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

  PointRenderResult() : geo::LaserRangeFinder::RenderResult(dummy_ranges_) {}

  void renderPoint(int index, float depth)
  {
    float old_depth = depth_;
    if (old_depth == 0 || depth < old_depth)
    {
      depth_ = depth;
      entity_ = active_entity_;
    }
  }

  std::vector<double> dummy_ranges_;

  std::string active_entity_;

  double depth_;
  std::string entity_;


};

RayTraceResult ray_trace(const ed::WorldModel& world, const geo::Pose3D& raytrace_pose)
{
  PointRenderResult res;

  geo::LaserRangeFinder lrf;
  lrf.setAngleLimits(-1e-3, 1e-3);
  lrf.setNumBeams(1);
  lrf.setRangeLimits(0, 10);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Raytrace for each entity in the wm

  for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
  {
    const ed::EntityConstPtr& e = *it;

    if (!e->shape() || !e->has_pose())
      continue;

    for (const auto& volume : e->volumes())
    {
      res.active_entity_ = e->id().str();
      std::string name = volume.first;
      geo::ShapeConstPtr shape = volume.second;
      if (name == "on_top_of")
      {
        ROS_INFO("Raytrace on_top_of array of %s included", e->id().c_str());
        geo::LaserRangeFinder::RenderOptions opt;
        opt.setMesh(shape->getMesh(), raytrace_pose.inverse() * e->pose());

        lrf.render(opt, res);
      }
    }

    geo::LaserRangeFinder::RenderOptions opt;
//    opt.setMesh(e->shape()->getBoundingBox().getMesh(), raytrace_pose.inverse() * e->pose()); // Use bbx
    opt.setMesh(e->shape()->getMesh(), raytrace_pose.inverse() * e->pose()); // Use mesh

    lrf.render(opt, res);
  }

  geo::Vec3d point_sensor_frame(res.depth_, 0, 0);

  RayTraceResult result;
  result.entity_id_ = res.entity_;
  result.intersection_point_ = raytrace_pose * point_sensor_frame;
  result.succes_ = true;

  return result;
}

}

// ----------------------------------------------------------------------------------------------------

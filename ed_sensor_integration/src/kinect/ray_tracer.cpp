#include "ray_tracer.h"

#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/world_model.h>

#include <geolib/Shape.h>

#include <geolib/sensors/LaserRangeFinder.h>

#include <ros/console.h>

namespace ed_ray_tracer
{

// ----------------------------------------------------------------------------------------------------

class PointRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    PointRenderResult() : geo::LaserRangeFinder::RenderResult(dummy_ranges_), depth_(0.0), entity_("") {}

    void renderPoint(uint /*index*/, float depth)
    {
        float old_depth = depth_;
        if (old_depth == 0.0 || depth < old_depth)
        {
            ROS_DEBUG_STREAM("Using " << active_entity_ << " as active entity, depth from " << old_depth << " to " << depth);
            depth_ = depth;
            entity_ = active_entity_;
        }
        else
        {
            ROS_DEBUG_STREAM("Not using entity " << active_entity_);
        }
    }

    std::vector<double> dummy_ranges_;

    std::string active_entity_;

    double depth_;
    std::string entity_;

};

RayTraceResult ray_trace(const ed::WorldModel& world, const geo::Pose3D& raytrace_pose)
{
    ed::ErrorContext errc("ray_trace");
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

        if (!e->visual() || !e->has_pose())
            continue;

        res.active_entity_ = e->id().str();
        for (const auto& volume : e->volumes())
        {
            const std::string& name = volume.first;
            const geo::ShapeConstPtr& shape = volume.second;
            if (name == "on_top_of")
            {
                ROS_DEBUG("Raytrace on_top_of array of %s included", e->id().c_str());
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(shape->getMesh(), raytrace_pose.inverse() * e->pose());

                lrf.render(opt, res);
            }
        }

        ROS_DEBUG_STREAM("Raytracing to " << e->id() << " mesh");
        geo::LaserRangeFinder::RenderOptions opt;
        opt.setMesh(e->collision()->getMesh(), raytrace_pose.inverse() * e->pose()); // Use mesh

        lrf.render(opt, res);
    }

    geo::Vec3d point_sensor_frame(res.depth_, 0, 0);

    RayTraceResult result;

    if (res.entity_.empty())
    {
        ROS_DEBUG("Did not raytrace through any entity");
    }
    else
    {
        result.entity_id_ = res.entity_;
        result.intersection_point_ = raytrace_pose * point_sensor_frame;
        result.success_ = true;
    }

    return result;
}

}

// ----------------------------------------------------------------------------------------------------

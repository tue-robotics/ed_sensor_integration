#include "ed_sensor_integration/kinect/almodules/world_model_renderer.h"
#include <ed/entity.h>
#include <ed/world_model.h>

#include <geolib/Shape.h>

#include <ed/world_model/transform_crawler.h>
#include <ed/rgbd_data.h>

namespace edKinect
{

// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult {

public:

    SampleRenderResult(cv::Mat& z_buffer,
                       int width,
                       int height,
                       const ed::Entity* e,
                       const geo::DepthCamera& rasterizer,
                       pcl::PointCloud<pcl::PointXYZ>& pc,
                       std::vector<const ed::Entity*>& entities,
                       std::vector<int>& triangle_indices,
                       float max_range)
        : geo::RenderResult(width, height), z_buffer_(z_buffer), entity_(e), rasterizer_(rasterizer), pc_(pc), entities_(entities), triangles_indices_(triangle_indices), max_range_(max_range)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;

            if (depth < max_range_)
            {

                pcl::PointXYZ p;
                p.x = rasterizer_.project2Dto3DX(x) * depth;
                p.y = rasterizer_.project2Dto3DY(y) * depth;
                p.z = -depth;

                pc_.push_back(p);
                entities_.push_back(entity_);
                triangles_indices_.push_back(i_triangle);
            }
        }
    }

protected:

    cv::Mat z_buffer_;
    const ed::Entity* entity_;
    const geo::DepthCamera& rasterizer_;
    pcl::PointCloud<pcl::PointXYZ>& pc_;
    std::vector<const ed::Entity*>& entities_;
    std::vector<int>& triangles_indices_;
    float max_range_;

};

// ----------------------------------------------------------------------------------------------------

WorldModelRenderer::WorldModelRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

WorldModelRenderer::~WorldModelRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

void WorldModelRenderer::render(const ed::RGBDData& sensor_data,
                                const ed::WorldModel &world_model,
                                float max_range,
                                const rgbd::View& view,
                                cv::Mat& img,
                                pcl::PointCloud<pcl::PointXYZ>& pc,
                                std::vector<const ed::Entity*>& pc_entity_ptrs)
{
    geo::Pose3D p_corr(geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1), geo::Vector3(0, 0, 0));

    std::set<ed::UUID> rendered_entities;

    std::string cam_id = sensor_data.image->getFrameId();
    if (cam_id[0] == '/')
        cam_id = cam_id.substr(1);

    for(ed::world_model::TransformCrawler tc(world_model, cam_id, sensor_data.image->getTimestamp()); tc.hasNext(); tc.next())
    {
        const ed::EntityConstPtr& e = tc.entity();
        if (e->shape())
        {
            rendered_entities.insert(e->id());

            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), p_corr * tc.transform());

            std::vector<int> triangle_indices;
            SampleRenderResult res(img, view.getWidth(), view.getHeight(), e.get(), view.getRasterizer(), pc, pc_entity_ptrs, triangle_indices, max_range);

            // Render
            view.getRasterizer().render(opt, res);
        }
    }

    for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->shape() && e->has_pose() && rendered_entities.find(e->id()) == rendered_entities.end())
        {
            geo::Pose3D pose = sensor_data.sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            std::vector<int> triangle_indices;
            SampleRenderResult res(img, view.getWidth(), view.getHeight(), e.get(), view.getRasterizer(), pc, pc_entity_ptrs, triangle_indices, max_range);

            // Render
            view.getRasterizer().render(opt, res);
        }
    }
}

}

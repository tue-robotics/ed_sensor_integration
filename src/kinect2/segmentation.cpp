#include "segmentation.h"

#include <geolib/datatypes.h>
#include <geolib/ros/tf_conversions.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/integral_image_normal.h>

// Rendering
#include <ed/world_model/transform_crawler.h>

#include <pcl/filters/filter.h>

#include <ed/convex_hull_calc.h>

#include <ed/error_context.h>

// Association
#include "ed_sensor_integration/association_matrix.h"

// Meshing
#include "ed_sensor_integration/meshing.h"
#include <geolib/serialization.h>

// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer_, cv::Mat& normal_map_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), normal_map(normal_map_), i_normal_offset(0),
          in_view(false) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
            normal_map.at<int>(y, x) = i_normal_offset + i_triangle;
            in_view = true;
        }
    }

    cv::Mat& z_buffer;
    cv::Mat& normal_map;
    int i_normal_offset;
    bool in_view;

};

// ----------------------------------------------------------------------------------------------------

bool pointAssociates(const pcl::PointNormal& p, pcl::PointCloud<pcl::PointNormal>& pc, int x, int y, float& min_dist_sq)
{
    const pcl::PointNormal& p2 = pc.points[pc.width * y + x];

    float dx = p2.x - p.x;
    float dy = p2.y - p.y;
    float dz = p2.z - p.z;

    float dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq < min_dist_sq)
    {
        if (p.normal_x != p.normal_x)
        {
            // Point does not have normal
            min_dist_sq = dist_sq;
            return true;
        }

        // Check normals
        float dot = p.normal_x * p2.normal_x + p.normal_y * p2.normal_y + p.normal_z * p2.normal_z;
        if (dot > 0.8)  // TODO: magic number
        {
            min_dist_sq = dist_sq;
            return true;
        }
    }

    return false;
}

// ----------------------------------------------------------------------------------------------------

void Segmenter::segment(const ed::WorldModel& world, const rgbd::Image& image, const geo::Pose3D& sensor_pose, std::vector<Cluster>& segments)
{
    // - - - - - - - - - - - - - - - - - -
    // Downsample depth image

    int factor = 2;

    const cv::Mat& depth_original = image.getDepthImage();

    cv::Mat depth;
    if (factor == 1)
    {
        depth = depth_original;
    }
    else
    {
        depth = cv::Mat(depth_original.rows / factor, depth_original.cols / factor, CV_32FC1, 0.0);

        for(int y = 0; y < depth.rows; ++y)
        {
            for(int x = 0; x < depth.cols; ++x)
            {
                depth.at<float>(y, x) = depth_original.at<float>(y * factor, x * factor);
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Convert depth map to point cloud

    rgbd::View view(image, depth.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    pcl::PointCloud<pcl::PointNormal>::Ptr pc(new pcl::PointCloud<pcl::PointNormal>);
    pc->width = depth.cols;
    pc->height = depth.rows;
    pc->is_dense = false; // may contain NaNs

    unsigned int size = depth.cols * depth.rows;
    pc->points.resize(size);

    unsigned int i = 0;
    for(int y = 0; y < depth.rows; ++y)
    {
        for(int x = 0; x < depth.cols; ++x)
        {
            float d = depth.at<float>(i);
            pcl::PointNormal& p = pc->points[i];

            if (d > 0 && d == d)
            {
                p.x = cam_model.project2Dto3DX(x) * d;
                p.y = cam_model.project2Dto3DY(y) * d;
                p.z = d;
            }
            else
            {
                p.x = NAN;
                p.y = NAN;
                p.z = NAN;
            }

            ++i;
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Estimate sensor point cloud normals

    pcl::IntegralImageNormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(1.2f);
    ne.setNormalSmoothingSize(10.0f / factor);
    ne.setViewPoint(0, 0, 0);
    ne.setInputCloud(pc);
    ne.compute(*pc);

    // - - - - - - - - - - - - - - - - - -
    // Render world model and calculate normals

    cv::Mat depth_model(depth.rows, depth.cols, CV_32FC1, 0.0);
    cv::Mat normal_map(depth.rows, depth.cols, CV_32SC1, -1);
    std::vector<geo::Vector3> model_normals;

    SampleRenderResult res(depth_model, normal_map);

    geo::Pose3D p_corr(geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1), geo::Vector3(0, 0, 0));

    std::set<ed::UUID> rendered_entities;

    std::string cam_id = image.getFrameId();
    if (cam_id[0] == '/')
        cam_id = cam_id.substr(1);

    for(ed::world_model::TransformCrawler tc(world, cam_id, image.getTimestamp()); tc.hasNext(); tc.next())
    {
        const ed::EntityConstPtr& e = tc.entity();
        if (e->shape())
        {
            res.in_view = false;

            const geo::Mesh& mesh = e->shape()->getMesh();

            geo::Pose3D pose = p_corr * tc.transform();
            geo::RenderOptions opt;
            opt.setMesh(mesh, pose);

            // Render
            cam_model.render(opt, res);

            if (res.in_view)
            {
                const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
                const std::vector<geo::Vector3>& vertices = mesh.getPoints();

                for(unsigned int i = 0; i < triangles.size(); ++i)
                {
                    const geo::TriangleI& t = triangles[i];
                    const geo::Vector3& p1 = vertices[t.i1_];
                    const geo::Vector3& p2 = vertices[t.i2_];
                    const geo::Vector3& p3 = vertices[t.i3_];

                    // Calculate normal
                    geo::Vector3 n = ((p3 - p1).cross(p2 - p1)).normalized();

                    // Transform to camera frame
                    n = pose.R * n;

                    // Why is this needed? (geolib vs ROS frame?)
                    n.x = -n.x;
                    n.y = -n.y;

                    model_normals.push_back(n);
                }

                res.i_normal_offset += triangles.size();
            }

            rendered_entities.insert(e->id());
        }
    }

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() && e->has_pose() && rendered_entities.find(e->id()) == rendered_entities.end())
        {
            res.in_view = false;

            const geo::Mesh& mesh = e->shape()->getMesh();

            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(mesh, pose);

            // Render
            cam_model.render(opt, res);

            if (res.in_view)
            {
                const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
                const std::vector<geo::Vector3>& vertices = mesh.getPoints();

                for(unsigned int i = 0; i < triangles.size(); ++i)
                {
                    const geo::TriangleI& t = triangles[i];
                    const geo::Vector3& p1 = vertices[t.i1_];
                    const geo::Vector3& p2 = vertices[t.i2_];
                    const geo::Vector3& p3 = vertices[t.i3_];

                    // Calculate normal
                    geo::Vector3 n = ((p3 - p1).cross(p2 - p1)).normalized();

                    // Transform to camera frame
                    n = pose.R * n;

                    // Why is this needed? (geolib vs ROS frame?)
                    n.x = -n.x;
                    n.y = -n.y;

                    model_normals.push_back(n);
                }

                res.i_normal_offset += triangles.size();
            }
        }
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr pc_model(new pcl::PointCloud<pcl::PointNormal>);
    pc_model->points.resize(size);
    pc_model->width = depth_model.cols;
    pc_model->height = depth_model.rows;
    pc_model->is_dense = false; // may contain NaNs

    {
        unsigned int i = 0;
        for(int y = 0; y < depth_model.rows; ++y)
        {
            for(int x = 0; x < depth_model.cols; ++x)
            {
                float d = depth_model.at<float>(i);
                int i_normal = normal_map.at<int>(i);

                pcl::PointNormal& p = pc_model->points[i];

                if (d > 0)
                {
                    p.x = cam_model.project2Dto3DX(x) * d;
                    p.y = cam_model.project2Dto3DY(y) * d;
                    p.z = d;

                    // Set normal
                    const geo::Vector3& n = model_normals[i_normal];
                    p.normal_x = n.x;
                    p.normal_y = n.y;
                    p.normal_z = n.z;
                }
                else
                {
                    p.x = NAN;
                    p.y = NAN;
                    p.z = NAN;
                }

                ++i;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter sensor points that are too far or behind world model

    for(unsigned int i = 0; i < size; ++i)
    {
        pcl::PointNormal& ps = pc->points[i];
        const pcl::PointNormal& pm = pc_model->points[i];

        if (ps.x == ps.x)
        {
            if ((ps.z > max_range_) || (pm.x == pm.x && ps.z > pm.z))
                ps.x = NAN;
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Perform point normal association

    cv::Mat cluster_visited_map(depth.rows, depth.cols, CV_8UC1, cv::Scalar(1));
    std::vector<unsigned int> non_assoc_mask;

    for(int y = 0; y < depth.rows; ++y)
    {
        for(int x = 0; x < depth.cols; ++x)
        {
            unsigned int i = depth.cols * y + x;
            const pcl::PointNormal& p = pc->points[i];

            if (p.x != p.x)
                continue;

//            if (p.normal_x != p.normal_x)
//            {
//                // No normal, but we do have a depth measurement at this pixel. Make sure it
//                // can still be visited by the clustering algorithm
//                cluster_visited_map.at<unsigned char>(i) = 0;
//                continue;
//            }

            bool associates = false;
            float assoc_corr_dist = association_correspondence_distance_ * p.z;

            float min_dist_sq = assoc_corr_dist * assoc_corr_dist;

            int w = association_correspondence_distance_ * cam_model.getOpticalCenterX() / p.z;

            associates = pointAssociates(p, *pc_model, x, y, min_dist_sq);

            for(int d = 1; d < w && !associates; ++d)
            {
                associates =
                        (x - d >= 0 && pointAssociates(p, *pc_model, x - d, y, min_dist_sq)) ||
                        (x + d < depth.cols && pointAssociates(p, *pc_model, x + d, y, min_dist_sq)) ||
                        (y - d >= 0 && pointAssociates(p, *pc_model, x, y - d, min_dist_sq)) ||
                        (y + d < depth.rows && pointAssociates(p, *pc_model, x, y + d, min_dist_sq));

//                int x2 = x - d;
//                int y2 = y - d;

//                for(; !associates && x2 < x + d; ++x2)
//                    associates = associates || pointAssociates(p, *pc_model, x2, y2, min_dist_sq);

//                for(; !associates && y2 < y + d; ++y2)
//                    associates = associates || pointAssociates(p, *pc_model, x2, y2, min_dist_sq);

//                for(; !associates && x2 > x - d; --x2)
//                    associates = associates || pointAssociates(p, *pc_model, x2, y2, min_dist_sq);

//                for(; !associates && y2 > y - d; --y2)
//                    associates = associates || pointAssociates(p, *pc_model, x2, y2, min_dist_sq);
            }

            if (!associates)
            {
                non_assoc_mask.push_back(i);
                cluster_visited_map.at<unsigned char>(i) = 0;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Cluster residual points and calculate their convex hulls

    // Mark borders as visited
    for(int x = 0; x < depth.cols; ++x)
    {
        cluster_visited_map.at<unsigned char>(0, x) = 1;
        cluster_visited_map.at<unsigned char>(depth.rows - 1, x) = 1;
    }

    for(int y = 0; y < depth.rows; ++y)
    {
        cluster_visited_map.at<unsigned char>(y, 0) = 1;
        cluster_visited_map.at<unsigned char>(y, depth.cols - 1) = 1;
    }

    int dirs[] = { -1, 1, -depth.cols, depth.cols };

    for(unsigned int i = 0; i < non_assoc_mask.size(); ++i)
    {
        unsigned int p = non_assoc_mask[i];

        if (cluster_visited_map.at<unsigned char>(p) == 1)
            continue;

        // Create cluster
        segments.push_back(Cluster());
        Cluster& cluster = segments.back();

        cluster.image_mask.setSize(depth.cols, depth.rows);

        std::queue<unsigned int> Q;
        Q.push(p);

        // Mark visited
        cluster_visited_map.at<unsigned char>(p) = 1;

        while(!Q.empty())
        {
            unsigned int p1 = Q.front();
            Q.pop();

            float p1_d = depth.at<float>(p1);

            // Add to cluster
            cluster.pixels.push_back(p1);
            cluster.image_mask.addPoint(p1);

            for(int d = 0;  d < 4; ++d)
            {
                unsigned int p2 = p1 + dirs[d];
                float p2_d = depth.at<float>(p2);

                // If not yet visited, and depth is within bounds
                if (cluster_visited_map.at<unsigned char>(p2) == 0 && std::abs<float>(p2_d - p1_d) < 0.1)
                {
                    // Mark visited
                    cluster_visited_map.at<unsigned char>(p2) = 1;
                    Q.push(p2);
                }
            }
        }

        // Check if cluster has enough points. If not, remove it from the list
        if (cluster.pixels.size() < 100) // TODO: magic number
        {
            segments.pop_back();
            continue;
        }

        // Calculate cluster convex hull

        float z_min = 1e9;
        float z_max = -1e9;
        bool complete = true;

        // Calculate z_min and z_max of cluster
        std::vector<geo::Vec2f> points_2d(cluster.pixels.size());
        for(unsigned int j = 0; j < cluster.pixels.size(); ++j)
        {
            const pcl::PointNormal& p = pc->points[cluster.pixels[j]];

            // Transform sensor point to map frame
            geo::Vector3 p_map = sensor_pose * geo::Vector3(p.x, p.y, -p.z); //! Not a right handed frame?

            points_2d[j] = geo::Vec2f(p_map.x, p_map.y);

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);
        }

        ed::ConvexHull& cluster_chull = cluster.chull;
        geo::Pose3D& cluster_pose = cluster.pose;

        ed::convex_hull::create(points_2d, z_min, z_max, cluster_chull, cluster_pose);
        cluster_chull.complete = complete;
    }
}

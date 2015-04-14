#include "plugin.h"

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

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <pcl/filters/filter.h>

#include <ed/update_request.h>

// Properties
#include "ed_sensor_integration/properties/convex_hull_calc.h"
#include "ed_sensor_integration/properties/convex_hull_info.h"
#include "ed_sensor_integration/properties/pose_info.h"
#include "ed_sensor_integration/properties/counter_info.h"

#include <ed/measurement.h>
#include <ed/mask.h>

// Visualization
#include "visualization.h"

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
//    std::cout << "    " << x << ", " << y << std::endl;

    const pcl::PointNormal& p2 = pc.points[pc.width * y + x];

    float dx = p2.x - p.x;
    float dy = p2.y - p.y;
    float dz = p2.z - p.z;

    float dist_sq = dx * dx + dy * dy + dz * dz;

    if (dist_sq < min_dist_sq)
    {
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

ed::MeasurementPtr createMeasurement(const rgbd::ImageConstPtr& rgbd_image, const cv::Mat& depth,
                                     const geo::Pose3D& sensor_pose, const std::vector<unsigned int>& cluster)
{
    const cv::Mat& rgb = rgbd_image->getRGBImage();

    int f = rgb.cols / depth.cols;

    ed::ImageMask image_mask(rgb.cols, rgb.rows);
    for(unsigned int j = 0; j < cluster.size(); ++j)
    {
        int p = cluster[j];

        int x_depth = p % depth.cols;
        int y_depth = p / depth.cols;
        int x_rgb = x_depth * f;
        int y_rgb = y_depth * f;

        for(int x = x_rgb; x < x_rgb + f; ++x)
        {
            for(int y = y_rgb; y < y_rgb + f; ++y)
            {
                image_mask.addPoint(x, y);
//                std::cout << x << ", " << y << " (" << x_depth << ", " << y_depth << ")" << std::endl;
            }
        }
    }

    // Create measurement
    ed::MeasurementPtr m(new ed::Measurement(rgbd_image, image_mask, sensor_pose));


    return m;
}

// ----------------------------------------------------------------------------------------------------

void convertConvexHull(const ConvexHull& c, const geo::Pose3D& pose, ed::ConvexHull2D& c2)
{
    c2.min_z = c.z_min + pose.t.z;
    c2.max_z = c.z_max + pose.t.z;
    c2.center_point = pose.t;

    c2.chull.resize(c.points.size());
    for(unsigned int i = 0; i < c.points.size(); ++i)
        c2.chull.points[i] = pcl::PointXYZ(c.points[i].x + pose.t.x, c.points[i].y + pose.t.y, 0);
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin() : tf_listener_(0), debug_(false)
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    if (config.value("topic", topic_))
    {
        std::cout << "[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic_ << "'." << std::endl;
        kinect_client_.intialize(topic_);
    }

    config.value("max_correspondence_distance", association_correspondence_distance_);
    config.value("max_range", max_range_);

    if (config.value("debug", debug_, tue::OPTIONAL))
    {
        if (debug_)
            std::cout << "[ED KINECT PLUGIN] Debug print statements on" << std::endl;
    }

    tf_listener_ = new tf::TransformListener;

    xy_padding_ = 0.1; // TODO: Magic number
    z_padding_ = 0.1; // TODO: Magic number
    border_padding_ = 0.30; // TODO: Magic number
    assoc_clear_thr_ = 10; // TODO: magic number

    // Register properties
    init.properties.registerProperty("convex_hull", k_convex_hull_, new ConvexHullInfo);
    init.properties.registerProperty("pose", k_pose_, new PoseInfo);
    init.properties.registerProperty("clearing_counter", k_clearing_counter_, new CounterInfo);

    // Initialize image publishers for visualization
    viz_sensor_normals_.initialize("kinect/viz/sensor_normals");
    viz_model_normals_.initialize("kinect/viz/model_normals");
    viz_clusters_.initialize("kinect/viz/clusters");
    viz_update_req_.initialize("kinect/viz/update_request");
    viz_model_render_.initialize("kinect/viz/depth_render");
    viz_model_sensor_diff_.initialize("kinect/viz/model_sensor_diff");
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    tue::Timer t_total;
    t_total.start();

    const ed::WorldModel& world = data.world;

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_.nextImage();
    if (rgbd_image)
        image_buffer_.push(rgbd_image);

    if (image_buffer_.empty())
        return;

    rgbd_image = image_buffer_.front();


    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    geo::Pose3D sensor_pose;

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
        image_buffer_.pop();

    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
                image_buffer_.pop();
            else
                ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
            return;
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
        }
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
        return;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);


    // - - - - - - - - - - - - - - - - - -
    // Downsample depth image

    int factor = 2;

    const cv::Mat& depth_original = rgbd_image->getDepthImage();

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

    rgbd::View view(*rgbd_image, depth.cols);
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

    tue::Timer t_normal;
    t_normal.start();

    pcl::IntegralImageNormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f); // TODO: Magic number
    ne.setNormalSmoothingSize(10.0f / factor); // TODO: Magic number
    ne.setViewPoint(0, 0, 0);
    ne.setInputCloud(pc);
    ne.compute(*pc);

    if (debug_)
        std::cout << "Calculating normals took " << t_normal.getElapsedTimeInMilliSec() << " ms." << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Render world model and calculate normals

    tue::Timer t_render;
    t_render.start();

    cv::Mat depth_model(depth.rows, depth.cols, CV_32FC1, 0.0);
    cv::Mat normal_map(depth.rows, depth.cols, CV_32SC1, -1);
    std::vector<geo::Vector3> model_normals;

    SampleRenderResult res(depth_model, normal_map);

    geo::Pose3D p_corr(geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1), geo::Vector3(0, 0, 0));

    std::set<ed::UUID> rendered_entities;

    std::string cam_id = rgbd_image->getFrameId();
    if (cam_id[0] == '/')
        cam_id = cam_id.substr(1);

    for(ed::world_model::TransformCrawler tc(world, cam_id, rgbd_image->getTimestamp()); tc.hasNext(); tc.next())
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

    // Visualize depth model (if asked for)
    visualizeDepthImage(depth_model, viz_model_render_);

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

    if (debug_)
        std::cout << "Rendering (with normals) took " << t_render.getElapsedTimeInMilliSec() << " ms." << std::endl;

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

    tue::Timer t_assoc;
    t_assoc.start();

    cv::Mat cluster_visited_map(depth.rows, depth.cols, CV_8UC1, cv::Scalar(1));
    std::vector<unsigned int> non_assoc_mask;

    int w_max = 20; // TODO: magic number
    for(int y = w_max; y < depth.rows - w_max; ++y)
    {
        for(int x = w_max; x < depth.cols - w_max; ++x)
        {
            unsigned int i = depth.cols * y + x;
            const pcl::PointNormal& p = pc->points[i];

            if (p.x != p.x)
                continue;

            if (p.normal_x != p.normal_x)
            {
                // No normal, but we do have a depth measurement at this pixel. Make sure it
                // can still be visited by the clustering algorithm
                cluster_visited_map.at<unsigned char>(i) = 0;
                continue;
            }

            bool associates = false;
            float min_dist_sq = association_correspondence_distance_ * association_correspondence_distance_;

            int w = std::min<int>(association_correspondence_distance_ * cam_model.getOpticalCenterX() / p.z, w_max);

            associates = pointAssociates(p, *pc_model, x, y, min_dist_sq);

            for(int d = 1; d < w && !associates; ++d)
            {
                associates =
                        pointAssociates(p, *pc_model, x - d, y, min_dist_sq) ||
                        pointAssociates(p, *pc_model, x + d, y, min_dist_sq) ||
                        pointAssociates(p, *pc_model, x, y - d, min_dist_sq) ||
                        pointAssociates(p, *pc_model, x, y + d, min_dist_sq);

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

    if (debug_)
        std::cout << "Point association took " << t_assoc.getElapsedTimeInMilliSec() << " ms." << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Cluster residual points

    tue::Timer t_clustering;
    t_clustering.start();

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

    std::vector<std::vector<unsigned int> > clusters;

    int dirs[] = { -1, 1, -depth.cols, depth.cols };

    for(unsigned int i = 0; i < non_assoc_mask.size(); ++i)
    {
        unsigned int p = non_assoc_mask[i];

        if (cluster_visited_map.at<unsigned char>(p) == 1)
            continue;

        // Create cluster
        clusters.push_back(std::vector<unsigned int>());
        std::vector<unsigned int>& cluster = clusters.back();

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
            cluster.push_back(p1);

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
        if (cluster.size() < 30)
            clusters.pop_back();
    }

    if (debug_)
        std::cout << "Clustering took " << t_clustering.getElapsedTimeInMilliSec() << " ms." << std::endl;


    // - - - - - - - - - - - - - - - - - -
    // Calculate cluster convex hulls and check collisions

    tue::Timer t_chull;
    t_chull.start();

    std::set<ed::UUID> associated_ids;

    for(std::vector<std::vector<unsigned int> > ::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        const std::vector<unsigned int>& cluster = *it;

        float z_min = 1e9;
        float z_max = -1e9;
        bool complete = true;

        // Calculate z_min and z_max of cluster
        std::vector<geo::Vec2f> points_2d(cluster.size());
        for(unsigned int j = 0; j < cluster.size(); ++j)
        {
            const pcl::PointNormal& p = pc->points[cluster[j]];

            // Transform sensor point to map frame
            geo::Vector3 p_map = sensor_pose * geo::Vector3(p.x, p.y, -p.z); //! Not a right handed frame?

            points_2d[j] = geo::Vec2f(p_map.x, p_map.y);

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);

            // If cluster is completely within a frame inside the view, it is called complete
            if ( p.x <    border_padding_  * view.getWidth() && p.y <    border_padding_  * view.getHeight() &&
                 p.x > (1-border_padding_) * view.getWidth() && p.y > (1-border_padding_) * view.getHeight() )
            {
                complete = false;
            }
        }

        ConvexHull cluster_chull;
        geo::Pose3D cluster_pose;
        convex_hull::create(points_2d, z_min, z_max, cluster_chull, cluster_pose);
        cluster_chull.complete = complete;

        // Check for collisions with convex hulls of existing entities
        bool associated = false;
        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape())
                continue;

            const geo::Pose3D* entity_pose = e->property(k_pose_);
            const ConvexHull* entity_chull = e->property(k_convex_hull_);

            // Check if the convex hulls collide
            if (entity_pose && entity_chull
                    && convex_hull::collide(*entity_chull, entity_pose->t, cluster_chull, cluster_pose.t, xy_padding_, z_padding_))
            {
                associated = true;
                req.setProperty(e->id(), k_clearing_counter_, 0);

                // Update pose and convex hull
                std::vector<geo::Vec2f> new_points_MAP;
                for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull->points.begin(); p_it != entity_chull->points.end(); ++p_it)
                {
                    geo::Vec2f p_chull_MAP(p_it->x + entity_pose->t.x, p_it->y + entity_pose->t.y);

                    // Calculate the 3d coordinate of entity chull points in absolute frame, in the middle of the rib
                    geo::Vector3 p_rib(p_chull_MAP.x, p_chull_MAP.y, entity_pose->t.z);

                    // Transform to the sensor frame
                    geo::Vector3 p_rib_cam = sensor_pose.inverse() * p_rib;

                    // Project to image frame
                    cv::Point2d p_2d = view.getRasterizer().project3Dto2D(p_rib_cam);

                    // Check if the point is in view, and is not occluded by sensor points
                    if (p_2d.x > 0 && p_2d.y > 0 && p_2d.x < view.getWidth() && p_2d.y < view.getHeight() && !entity_chull->complete)
                    {
                        // Only add old entity chull point if depth from sensor is now zero, entity point is out of range or depth from sensor is smaller than depth of entity point
                        float dp = -p_rib_cam.z;
                        float ds = depth.at<float>(p_2d);
                        if (ds == 0 || dp > max_range_ || dp > ds)
                            new_points_MAP.push_back(p_chull_MAP);
                    }
                    else
                    {
                        new_points_MAP.push_back(p_chull_MAP);
                    }
                }

                if ( !entity_chull->complete )
                {
                    // Add the points of the new convex hull.
                    for(std::vector<geo::Vec2f>::const_iterator p_it = cluster_chull.points.begin(); p_it != cluster_chull.points.end(); ++p_it)
                    {
                        new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster_pose.t.x, p_it->y + cluster_pose.t.y));
                    }
                }
                // TODO: Else: Remove overlap with complete entity chull from cluster, threshold on size and add cluster to cluster list to be associated with something else?

                // And calculate the convex hull of these points
                ConvexHull new_chull;
                geo::Pose3D new_pose;

                // (TODO: taking the z_min and z_max of chull is quite arbitrary...)
                if (cluster_chull.complete)
                {
                    z_min = entity_chull->z_min + cluster_pose.t.z;
                    z_max = entity_chull->z_max + cluster_pose.t.z;
                }
                convex_hull::create(new_points_MAP, z_min, z_max, new_chull, new_pose);
                new_chull.complete = cluster_chull.complete;

                req.setProperty(e->id(), k_pose_, new_pose);
                req.setProperty(e->id(), k_convex_hull_, new_chull);

                // Set old chull (is used in other plugins, e.g. navigation)
                ed::ConvexHull2D chull_old;
                convertConvexHull(new_chull, new_pose, chull_old);
                req.setConvexHull(e->id(), chull_old);
                req.setPose(e->id(), new_pose);

                // Add measurement
                req.addMeasurement(e->id(), createMeasurement(rgbd_image, depth, sensor_pose, cluster));

                associated_ids.insert(e->id());

                break;
            }
        }


        // Check for collisions with convex hulls of existing entities
        if (!associated)
        {
            for(std::vector<EntityPtr>::iterator le_it = local_entities_.begin(); le_it != local_entities_.end(); ++le_it)
            {
                EntityPtr e = *le_it;

                geo::Pose3D entity_pose = e->pose;
                ConvexHull entity_chull = e->chull;

                // Check if the convex hulls collide
                if (convex_hull::collide(entity_chull, entity_pose.t, cluster_chull, cluster_pose.t, xy_padding_, z_padding_))
                {
                    associated = true;
                    e->count++;

                    // Update pose and convex hull
                    std::vector<geo::Vec2f> new_points_MAP;
                    std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin();

                    for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
                    {
                        geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

                        // Calculate the 3d coordinate of entity chull points in absolute frame, in the middle of the rib
                        geo::Vector3 p_rib(p_chull_MAP.x, p_chull_MAP.y, entity_pose.t.z);

                        // Transform to the sensor frame
                        geo::Vector3 p_rib_cam = sensor_pose.inverse() * p_rib;

                        // Project to image frame
                        cv::Point2d p_2d = view.getRasterizer().project3Dto2D(p_rib_cam);

                        // Check if the point is in view, and is not occluded by sensor points. Only update chull if not complete
                        if (p_2d.x > 0 && p_2d.y > 0 && p_2d.x < view.getWidth() && p_2d.y < view.getHeight() && !entity_chull.complete)
                        {
                            // Only add old entity chull point if depth from sensor is now zero, entity point is out of range or depth from sensor is smaller than depth of entity point
                            float dp = -p_rib_cam.z;
                            float ds = depth.at<float>(p_2d);
                            if (ds == 0 || dp > max_range_ || dp > ds)
                                new_points_MAP.push_back(p_chull_MAP);
                        }
                        else
                        {
                            new_points_MAP.push_back(p_chull_MAP);
                        }
                    }

                    // Add points to chull (only if not complete)
                    if ( !entity_chull.complete )
                    {
                        for(std::vector<geo::Vec2f>::const_iterator p_it = cluster_chull.points.begin(); p_it != cluster_chull.points.end(); ++p_it)
                        {
                            new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster_pose.t.x, p_it->y + cluster_pose.t.y));
                        }
                    }

                    // And calculate the convex hull of these points
                    ConvexHull new_chull;
                    geo::Pose3D new_pose;

                    // (TODO: taking the z_min and z_max of chull is quite arbitrary...)
                    if (cluster_chull.complete)
                    {
                        z_min = entity_chull.z_min + cluster_pose.t.z;
                        z_max = entity_chull.z_max + cluster_pose.t.z;
                    }
                    convex_hull::create(new_points_MAP, z_min, z_max, new_chull, new_pose);
                    new_chull.complete = cluster_chull.complete;

                    e->pose = new_pose;
                    e->chull = new_chull;

                    // Add measurement
                    e->measurements.push_back(createMeasurement(rgbd_image, depth, sensor_pose, cluster));

                    associated_ids.insert(e->id);

                    break;
                }
            }
        }

        // ---------------------------------------------------------------------------------------------------------------

        if (!associated)
        {
            // Add new entity
            ed::UUID id = ed::Entity::generateID();

            // Set old chull (is used in other plugins, e.g. navigation)
            ed::ConvexHull2D chull_old;
            convertConvexHull(cluster_chull, cluster_pose, chull_old);

            EntityPtr e(new Entity);
            e->id = id;
            e->pose = cluster_pose;
            e->chull = cluster_chull;
            e->measurements.push_back(createMeasurement(rgbd_image, depth, sensor_pose, cluster));
            e->count = 1;
            local_entities_.push_back(e);
        }
    }

    if (debug_)
        std::cout << "Convex hull association took " << t_chull.getElapsedTimeInMilliSec() << " ms." << std::endl;

    std::vector<EntityPtr>::iterator le_it = local_entities_.begin();
    while (le_it != local_entities_.end())
    {
        EntityPtr e = *le_it;
        if ( e->count >= assoc_clear_thr_ )
        {
            // Create ed entity from local entity and add to update request
            // Also remove local entity from list
            req.setProperty(e->id, k_pose_, e->pose);
            req.setProperty(e->id, k_convex_hull_, e->chull);

            // Set old chull (is used in other plugins, e.g. navigation)
            ed::ConvexHull2D chull_old;
            convertConvexHull(e->chull, e->pose, chull_old);
            req.setConvexHull(e->id, chull_old);
            req.setPose(e->id, e->pose);

            // Add measurements
            for ( std::vector<ed::MeasurementPtr>::iterator m = e->measurements.begin(); m!=e->measurements.end(); ++m )
                req.addMeasurement(e->id, *m);

            le_it = local_entities_.erase(le_it);
        }
        else
            le_it++;
    }


    // - - - - - - - - - - - - - - - - - -
    // Clear unassociated clusters in view

    tue::Timer t_clear;
    t_clear.start();

    for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
    {
        const ed::EntityConstPtr& e = *e_it;
        if (e->shape() || associated_ids.find(e->id()) != associated_ids.end())
            continue;

        const geo::Pose3D* pose = e->property(k_pose_);
        const ConvexHull* chull = e->property(k_convex_hull_);

        if (!pose || !chull)
            continue;

        geo::Vector3 p = sensor_pose.inverse() * pose->t;

        cv::Point2d p_2d = cam_model.project3Dto2D(p);

        float clearing_padding_fraction = 0.2; // TODO: magic numbers

        if ( p_2d.x > depth.cols*clearing_padding_fraction && p_2d.x < depth.cols*(1.0-clearing_padding_fraction) &&
             p_2d.y > depth.rows*clearing_padding_fraction && p_2d.y < depth.rows*(1.0-clearing_padding_fraction) )
        {
            float d = depth.at<float>(p_2d);
            if (d > 0 && -p.z < d)
            {
                const int* count = e->property(k_clearing_counter_);
                if (count)
                {
                    if ( *count >= assoc_clear_thr_ )
                        req.removeEntity(e->id());
                    else
                        req.setProperty(e->id(), k_clearing_counter_, *count+1);
                }
                else
                    req.setProperty(e->id(), k_clearing_counter_, 1);
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Clear unassociated local entities in view
    std::vector<EntityPtr>::iterator e_it = local_entities_.begin();
    while (e_it != local_entities_.end())
    {
        EntityPtr e = *e_it;

        geo::Pose3D pose = e->pose;

        geo::Vector3 p = sensor_pose.inverse() * pose.t;

        cv::Point2d p_2d = cam_model.project3Dto2D(p);

        float clearing_padding_fraction = 0.2; // TODO: magic numbers

        if( p_2d.x > depth.cols*clearing_padding_fraction && p_2d.x < depth.cols*(1.0-clearing_padding_fraction) &&
            p_2d.y > depth.rows*clearing_padding_fraction && p_2d.y < depth.rows*(1.0-clearing_padding_fraction) )
        {
            float d = depth.at<float>(p_2d);
            if (d > 0 && -p.z < d)
                e->count--;
            if (e->count <= 0)
                e_it = local_entities_.erase(e_it);
            else
                ++e_it;
        }
        else
            ++e_it;
    }

    if (debug_)
    {
        std::cout << "Current size of the internal entity list is " << local_entities_.size() << std::endl;
        std::cout << "Clearing took " << t_clear.getElapsedTimeInMilliSec() << " ms." << std::endl;
        std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl << std::endl;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Visualize (will only send out images if someone's listening to them)

    visualizeNormals(*pc, viz_sensor_normals_);
    visualizeNormals(*pc_model, viz_model_normals_);
    visualizeDepthImage(depth-depth_model,viz_model_sensor_diff_);
    visualizeClusters(depth, clusters, viz_clusters_);
    visualizeUpdateRequest(world, req, rgbd_image, viz_update_req_);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

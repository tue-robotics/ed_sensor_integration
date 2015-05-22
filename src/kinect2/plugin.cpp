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

#include <ed/convex_hull_calc.h>

#include <ed/measurement.h>
#include <ed/mask.h>

// Visualization
#include "visualization.h"

#include <ed/error_context.h>

// Association
#include "ed_sensor_integration/association_matrix.h"

// Meshing
#include "ed_sensor_integration/meshing.h"
#include <geolib/serialization.h>


// ----------------------------------------------------------------------------------------------------

struct Cluster
{
    std::vector<unsigned int> mask;
    ed::ConvexHull chull;
    geo::Pose3D pose;
};

// ----------------------------------------------------------------------------------------------------

geo::Vector3 min(const geo::Vector3& p1, const geo::Vector3& p2)
{
    return geo::Vector3(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
}

// ----------------------------------------------------------------------------------------------------

geo::Vector3 max(const geo::Vector3& p1, const geo::Vector3& p2)
{
    return geo::Vector3(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
}

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

ed::MeasurementPtr createMeasurement(const rgbd::ImageConstPtr& rgbd_image, const cv::Mat& depth,
                                     const geo::Pose3D& sensor_pose, const std::vector<unsigned int>& cluster)
{
    ed::ImageMask image_mask(depth.cols, depth.rows);
    for(unsigned int j = 0; j < cluster.size(); ++j)
    {
        int p = cluster[j];

        int x = p % depth.cols;
        int y = p / depth.cols;

        image_mask.addPoint(x, y);
    }

    // Create measurement
    ed::MeasurementPtr m(new ed::Measurement(rgbd_image, image_mask, sensor_pose));

    return m;
}

// ----------------------------------------------------------------------------------------------------

void convertConvexHull(const ed::ConvexHull& c, const geo::Pose3D& pose, ed::ConvexHull2D& c2)
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

    xy_padding_ = 0.1;
    z_padding_ = 0.1;
    border_padding_ = 0.1;

    // Initialize image publishers for visualization
    viz_sensor_normals_.initialize("kinect/viz/sensor_normals");
    viz_model_normals_.initialize("kinect/viz/model_normals");
    viz_clusters_.initialize("kinect/viz/clusters");
    viz_update_req_.initialize("kinect/viz/update_request");
    viz_model_render_.initialize("kinect/viz/depth_render");

    // Initialize lock entity server
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_lock_entities_ = nh.advertiseService("kinect/lock_entities", &KinectPlugin::srvLockEntities, this);
    srv_mesh_entity_in_view_ = nh.advertiseService("kinect/mesh_entity_in_view", &KinectPlugin::srvMeshEntityInView, this);
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    tue::Timer t_total;
    t_total.start();

    const ed::WorldModel& world = data.world;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    cb_queue_.callAvailable();

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
            // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
            // to new, respectively). If it is too old, discard it.

            tf::StampedTransform latest_sensor_pose;
            tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
            // If image time stamp is older than latest transform, throw it out
            if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
            {
                image_buffer_.pop();
                ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                << ros::Time(rgbd_image->getTimestamp()));
            }

            return;
        }
        catch(tf::TransformException& exc)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
            return;
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
    ne.setMaxDepthChangeFactor(1.2f);
    ne.setNormalSmoothingSize(10.0f / factor);
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

    if (debug_)
        std::cout << "Point association took " << t_assoc.getElapsedTimeInMilliSec() << " ms." << std::endl;

    // - - - - - - - - - - - - - - - - - -
    // Cluster residual points and calculate their convex hulls

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

    std::vector<Cluster> clusters;

    int dirs[] = { -1, 1, -depth.cols, depth.cols };

    for(unsigned int i = 0; i < non_assoc_mask.size(); ++i)
    {
        unsigned int p = non_assoc_mask[i];

        if (cluster_visited_map.at<unsigned char>(p) == 1)
            continue;

        // Create cluster
        clusters.push_back(Cluster());
        Cluster& cluster = clusters.back();

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
            cluster.mask.push_back(p1);

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
        if (cluster.mask.size() < 100) // TODO: magic number
        {
            clusters.pop_back();
            continue;
        }

        // Calculate cluster convex hull

        float z_min = 1e9;
        float z_max = -1e9;
        bool complete = true;

        // Calculate z_min and z_max of cluster
        std::vector<geo::Vec2f> points_2d(cluster.mask.size());
        for(unsigned int j = 0; j < cluster.mask.size(); ++j)
        {
            const pcl::PointNormal& p = pc->points[cluster.mask[j]];

            // Transform sensor point to map frame
            geo::Vector3 p_map = sensor_pose * geo::Vector3(p.x, p.y, -p.z); //! Not a right handed frame?

            points_2d[j] = geo::Vec2f(p_map.x, p_map.y);

            z_min = std::min<float>(z_min, p_map.z);
            z_max = std::max<float>(z_max, p_map.z);

            int x_pixel = cluster.mask[j] % depth.cols;
            int y_pixel = cluster.mask[j] / depth.cols;

            // If cluster is completely within a frame inside the view, it is called complete
            if ( x_pixel <    border_padding_  * view.getWidth() || y_pixel <    border_padding_  * view.getHeight() ||
                 x_pixel > (1-border_padding_) * view.getWidth() || y_pixel > (1-border_padding_) * view.getHeight() )
            {
                complete = false;
            }
        }

        ed::ConvexHull& cluster_chull = cluster.chull;
        geo::Pose3D& cluster_pose = cluster.pose;

        ed::convex_hull::create(points_2d, z_min, z_max, cluster_chull, cluster_pose);
        cluster_chull.complete = complete;

//        if (cluster_chull.height() < 0.02 || cluster_chull.area < 0.015 * 0.015)  // TODO: robocup hack
//            clusters.pop_back();
    }

    if (debug_)
        std::cout << "Clustering took " << t_clustering.getElapsedTimeInMilliSec() << " ms." << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (!mesh_request_.id.empty())
    {
        const Cluster* biggest_cluster = 0;

        for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
        {
            const Cluster& cluster = *it;
            if (!biggest_cluster || cluster.mask.size() > biggest_cluster->mask.size())
            {
                biggest_cluster = &cluster;
            }
        }

        if (biggest_cluster && biggest_cluster->mask.size() > 500) // TODO: magic number
        {
            cv::Mat masked_depth_img(depth.rows, depth.cols, CV_32FC1, cv::Scalar(0));
            for(unsigned int i = 0; i < biggest_cluster->mask.size(); ++i)
                masked_depth_img.at<float>(biggest_cluster->mask[i]) = depth.at<float>(biggest_cluster->mask[i]);

            geo::Mesh mesh;
            ed_sensor_integration::depthImageToMesh(masked_depth_img, view.getRasterizer(), 0.05, mesh);

            // Transform mesh to map frame
            mesh = mesh.getTransformed(sensor_pose);

            // Determine bounding box
            const std::vector<geo::Vector3>& points = mesh.getPoints();
            geo::Vector3 points_min = points.front();
            geo::Vector3 points_max = points.front();
            for(std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
            {
                points_min = min(points_min, *it);
                points_max = max(points_max, *it);
            }

            // Determine origin
            geo::Vector3 origin = (points_min + points_max) / 2;

            geo::Transform transform(geo::Matrix3::identity(), -origin);
            mesh = mesh.getTransformed(transform);

            geo::ShapePtr shape(new geo::Shape);
            shape->setMesh(mesh);

            ed::UUID id = mesh_request_.id;
            req.setShape(id, shape);
            req.setPose(id, geo::Pose3D(geo::Matrix3::identity(), origin));

            if (!mesh_request_.type.empty())
                req.setType(id, mesh_request_.type);
        }

        mesh_request_.id.clear();
    }

    // - - - - - - - - - - - - - - - - - -
    // Convex hull association

    tue::Timer t_chull_association;
    t_chull_association.start();
    ed::ErrorContext errc("Convex hull assocation");

    float max_dist = 0.3;

    // Create selection of world model entities that could associate

    std::vector<ed::EntityConstPtr> entities;
    std::vector<int> entities_associated;

    if (!clusters.empty())
    {
        geo::Vector3 area_min = clusters[0].pose.t;
        geo::Vector3 area_max = clusters[0].pose.t;
        for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
        {
            const Cluster& cluster = *it;           
            area_min = min(area_min, cluster.pose.t);
            area_max = max(area_max, cluster.pose.t);
        }

        area_min -= geo::Vector3(max_dist, max_dist, max_dist);
        area_max += geo::Vector3(max_dist, max_dist, max_dist);

        for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
        {
            const ed::EntityConstPtr& e = *e_it;
            if (e->shape() || !e->has_pose())
                continue;

            const geo::Pose3D& entity_pose = e->pose();
            const ed::ConvexHull& entity_chull = e->convexHull();

            if (entity_chull.points.empty())
                continue;

            if (entity_pose.t.x < area_min.x || entity_pose.t.x > area_max.x
                    || entity_pose.t.y < area_min.y || entity_pose.t.y > area_max.y
                    || entity_pose.t.z < area_min.z || entity_pose.t.z > area_max.z)
            {
                continue;
            }

            entities.push_back(e);
        }

//        std::cout << "Clusters: " << clusters.size() << ", entities: " <<  entities.size() << std::endl;

        // Create association matrix
        ed_sensor_integration::AssociationMatrix assoc_matrix(clusters.size());
        for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
        {
            const Cluster& cluster = clusters[i_cluster];

            for (unsigned int i_entity = 0; i_entity < entities.size(); ++i_entity)
            {
                const ed::EntityConstPtr& e = entities[i_entity];

                const geo::Pose3D& entity_pose = e->pose();
                const ed::ConvexHull& entity_chull = e->convexHull();

                float dx = entity_pose.t.x - cluster.pose.t.x;
                float dy = entity_pose.t.y - cluster.pose.t.y;
                float dz = 0;

                if (entity_chull.z_max + entity_pose.t.z < cluster.chull.z_min + cluster.pose.t.z
                        || cluster.chull.z_max + cluster.pose.t.z < entity_chull.z_min + entity_pose.t.z)
                    // The convex hulls are non-overlapping in z
                    dz = entity_pose.t.z - cluster.pose.t.z;

                float dist_sq = (dx * dx + dy * dy + dz * dz);


                // TODO: better prob calculation
                double prob = 1.0 / (1.0 + 100 * dist_sq);

                double e_max_dist = 0.2;

                if (dist_sq > e_max_dist * e_max_dist)
                    prob = 0;

//                if (entity_chull.complete)
//                {
//                    // The idea: if the entity chull is complete, and the cluster chull is significantly taller,
//                    // the cluster can not be this entity
//                    if (cluster.chull.height() > 1.5 * entity_chull.height()) // TODO magic number
//                        prob = 0;
//                }
//                if (cluster.chull.complete)
//                {
//                    if (entity_chull.height() > 1.5 * cluster.chull.height()) // TODO magic number
//                        prob = 0;
//                }

                if (prob > 0)
                    assoc_matrix.setEntry(i_cluster, i_entity, prob);
            }
        }

        ed_sensor_integration::Assignment assig;
        if (!assoc_matrix.calculateBestAssignment(assig))
        {
            std::cout << "WARNING: Association failed!" << std::endl;
            return;
        }

        entities_associated.resize(entities.size(), -1);

        for (unsigned int i_cluster = 0; i_cluster < clusters.size(); ++i_cluster)
        {
            const Cluster& cluster = clusters[i_cluster];

            // Get the assignment for this cluster
            int i_entity = assig[i_cluster];

            ed::UUID id;
            ed::ConvexHull new_chull;
            geo::Pose3D new_pose;

            if (i_entity == -1)
            {
                // No assignment, so add as new cluster
                new_chull = cluster.chull;
                new_pose = cluster.pose;

                // Generate unique ID
                id = ed::Entity::generateID();

                // Update existence probability
                req.setExistenceProbability(id, 0.2); // TODO magic number
            }
            else
            {
                // Mark the entity as being associated
                entities_associated[i_entity] = i_cluster;

                // Update the entity
                const ed::EntityConstPtr& e = entities[i_entity];
                const ed::ConvexHull& entity_chull = e->convexHull();

                id = e->id();

                if (cluster.chull.complete)
                {
                    // Update the entity with the cluster convex hull (completely overriding the previous entity convex hull)
                    new_chull = cluster.chull;
                    new_pose = cluster.pose;
                }
                else if (entity_chull.complete)
                {
                    // Only update pose
                    new_chull = entity_chull;
                    new_pose = cluster.pose;
                }
                else
                {
                    const geo::Pose3D& entity_pose = e->pose();

                    // Calculate the combined z_min and z_max
                    double new_z_min = std::min(cluster.pose.t.z + cluster.chull.z_min, entity_pose.t.z + entity_chull.z_min);
                    double new_z_max = std::max(cluster.pose.t.z + cluster.chull.z_max, entity_pose.t.z + entity_chull.z_max);

                    // Create list of new convex hull points, in MAP frame
                    std::vector<geo::Vec2f> new_points_MAP;

                    // Add the points of the cluster
                    for(std::vector<geo::Vec2f>::const_iterator p_it = cluster.chull.points.begin(); p_it != cluster.chull.points.end(); ++p_it)
                        new_points_MAP.push_back(geo::Vec2f(p_it->x + cluster.pose.t.x, p_it->y + cluster.pose.t.y));

                    // Add the entity points that are still present in the depth map (or out of view)
                    for(std::vector<geo::Vec2f>::const_iterator p_it = entity_chull.points.begin(); p_it != entity_chull.points.end(); ++p_it)
                    {
                        geo::Vec2f p_chull_MAP(p_it->x + entity_pose.t.x, p_it->y + entity_pose.t.y);

                        // Calculate the 3d coordinate of entity chull points in absolute frame, in the middle of the rib
                        geo::Vector3 p_rib(p_chull_MAP.x, p_chull_MAP.y, (new_z_min + new_z_max) / 2);

                        // Transform to the sensor frame
                        geo::Vector3 p_rib_cam = sensor_pose.inverse() * p_rib;

                        // Project to image frame
                        cv::Point2d p_2d = view.getRasterizer().project3Dto2D(p_rib_cam);

                        // Check if the point is in view, and is not occluded by sensor points
                        if (p_2d.x > 0 && p_2d.y > 0 && p_2d.x < view.getWidth() && p_2d.y < view.getHeight())
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

                    // And calculate the convex hull of these points
                    ed::convex_hull::create(new_points_MAP, new_z_min, new_z_max, new_chull, new_pose);

                    if (cluster.chull.complete)
                        new_chull.complete = true;
                }

                // Update existence probability
                double p_exist = e->existenceProbability();
                req.setExistenceProbability(e->id(), std::min(1.0, p_exist + 0.1)); // TODO: very ugly prob update
            }            

            // Set convex hull and pose
            if (!new_chull.points.empty())
            {
                req.setConvexHullNew(id, new_chull, new_pose, rgbd_image->getTimestamp(), rgbd_image->getFrameId());
            }

            // Set timestamp
            req.setLastUpdateTimestamp(id, rgbd_image->getTimestamp());

            // Add measurement
            req.addMeasurement(id, createMeasurement(rgbd_image, depth, sensor_pose, cluster.mask));
        }

        if (debug_)
            std::cout << "Convex hull association took " << t_chull_association.getElapsedTimeInMilliSec() << " ms." << std::endl;
    }

    // - - - - - - - - - - - - - - - - - -
    // Clear unassociated entities in view

    tue::Timer t_clear;
    t_clear.start();
    errc.change("Clear entities");

//    for(unsigned int i = 0; i < entities_associated.size(); ++i)
//    {
//        const ed::EntityConstPtr& e = entities[i];

//        // If the entity is associated, skip it
//        if (entities_associated[i] >= 0)
//            continue;

//        const geo::Pose3D& pose = e->pose();

//        geo::Vector3 p = sensor_pose.inverse() * pose.t;

//        cv::Point2d p_2d = cam_model.project3Dto2D(p);

//        if ( p_2d.x >    border_padding_  * view.getWidth() && p_2d.y >    border_padding_  * view.getHeight() &&
//             p_2d.x < (1-border_padding_) * view.getWidth() && p_2d.y < (1-border_padding_) * view.getHeight() )
//        {
//            float d = depth.at<float>(p_2d);
//            if (d > 0 && -p.z < d)
//            {
////                std::cout << view.getWidth() << " x " << view.getHeight() << std::endl;
////                std::cout << "Should see " << e->id() << ", but I do not (" << p_2d << ")" << std::endl;

//                double p_exist = e->existenceProbability();
//                if (p_exist < 0.3) // TODO: magic number
//                    req.removeEntity(e->id());
//                else
//                {
//                    req.setExistenceProbability(e->id(), std::max(0.0, p_exist - 0.3));  // TODO: very ugly prob update
//                }
//            }
//        }
//    }

    if (debug_)
        std::cout << "Clearing took " << t_clear.getElapsedTimeInMilliSec() << " ms." << std::endl;

    if (debug_)
        std::cout << "Total took " << t_total.getElapsedTimeInMilliSec() << " ms." << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Visualize (will only send out images if someone's listening to them)

    visualizeNormals(*pc, viz_sensor_normals_);
    visualizeNormals(*pc_model, viz_model_normals_);
//    visualizeClusters(depth, clusters, viz_clusters_);
    visualizeUpdateRequest(world, req, rgbd_image, viz_update_req_);
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvLockEntities(ed_sensor_integration::LockEntities::Request& req, ed_sensor_integration::LockEntities::Response& res)
{
//    for(std::vector<std::string>::const_iterator it = req.lock_ids.begin(); it != req.lock_ids.end(); ++it)
//    {
//        locked_entities_.insert(*it);
//        local_ids_.erase(*it);
//    }

//    for(std::vector<std::string>::const_iterator it = req.unlock_ids.begin(); it != req.unlock_ids.end(); ++it)
//        locked_entities_.erase(*it);

    // TODO

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvMeshEntityInView(ed_sensor_integration::MeshEntityInView::Request& req, ed_sensor_integration::MeshEntityInView::Response& res)
{
    mesh_request_ = req;
    res.succeeded = true;
    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

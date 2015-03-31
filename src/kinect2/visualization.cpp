#include "visualization.h"

#include <ed/world_model.h>
#include <ed/entity.h>

#include "ed_sensor_integration/properties/convex_hull.h"

// ----------------------------------------------------------------------------------------------------

void visualizeNormals(const pcl::PointCloud<pcl::PointNormal>& pc, ed::ImagePublisher& pub)
{
    if (!pub.enabled())
        return;

    cv::Mat canvas(pc.height, pc.width, CV_8UC3, cv::Scalar(0, 0, 0));

    for(unsigned int i = 0; i < pc.size(); ++i)
    {
        const pcl::PointNormal& n = pc.points[i];
        if (n.normal_x == n.normal_x)
        {
            int r = 255 * (n.normal_x + 1) / 2;
            int g = 255 * (n.normal_y + 1) / 2;
            int b = 255 * (n.normal_z + 1) / 2;

            canvas.at<cv::Vec3b>(i) = cv::Vec3b(b, g, r);
        }
    }

    pub.publish(canvas);
}

// ----------------------------------------------------------------------------------------------------

void visualizeClusters(const cv::Mat& depth, const std::vector<std::vector<unsigned int> >& clusters, ed::ImagePublisher& pub)
{
    if (!pub.enabled())
        return;

    cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
        const std::vector<unsigned int>& cluster = clusters[i];

        for(unsigned int j = 0; j < cluster.size(); ++j)
        {
            unsigned int idx = cluster[j];
            int c = depth.at<float>(idx) * 25;
            canvas.at<cv::Vec3b>(idx) = cv::Vec3b(c, c, c);
        }
    }

    pub.publish(canvas);
}

// ----------------------------------------------------------------------------------------------------

//void visualizeWorldModel(const ed::WorldModel& world, const geo::Pose3D& sensor_pose, const rgbd::View& view, ed::ImagePublisher& pub)
//{
//    if (!pub.enabled())
//        return;

//    cv::Mat canvas(view.getHeight(), view.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

//    for(ed::WorldModel::const_iterator e_it = world.begin(); e_it != world.end(); ++e_it)
//    {
//        const ed::EntityConstPtr& e = *e_it;
//        if (e->shape())
//            continue;

//        const geo::Pose3D* pose = e->property(k_pose_);
//        const ConvexHull* chull = e->property(k_convex_hull_);

//        if (!pose || !chull)
//            continue;

//        for(unsigned int i = 0; i < chull->points.size(); ++i)
//        {
//            unsigned int j = (i + 1) % chull->points.size();

//            const geo::Vec2f& p1c = chull->points[i];
//            const geo::Vec2f& p2c = chull->points[j];

//            geo::Vector3 p1 = sensor_pose.inverse() * geo::Vector3(p1c.x + pose->t.x, p1c.y + pose->t.y, chull->z_max + pose->t.z);
//            geo::Vector3 p2 = sensor_pose.inverse() * geo::Vector3(p2c.x + pose->t.x, p2c.y + pose->t.y, chull->z_max + pose->t.z);

//            cv::Point2d p1_2d = view.getRasterizer().project3Dto2D(p1);
//            cv::Point2d p2_2d = view.getRasterizer().project3Dto2D(p2);

//            cv::line(canvas, p1_2d, p2_2d, cv::Scalar(255, 0, 0));
//        }
//    }

//    pub.publish(canvas);
//}

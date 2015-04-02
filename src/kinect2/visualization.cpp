#include "visualization.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/measurement.h>

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

void visualizeUpdateRequest(const ed::WorldModel& world, const ed::UpdateRequest& req, ed::ImagePublisher& pub)
{
    if (!pub.enabled())
        return;

    cv::Mat canvas;
    const cv::Mat* rgb = 0;

    for(std::map<ed::UUID, std::vector<ed::MeasurementConstPtr> >::const_iterator it = req.measurements.begin(); it != req.measurements.end(); ++it)
    {
        cv::Point2i p_top_left(1e6, 1e6);
        cv::Point2i p_bottom_right(-1e6, -1e6);

        const std::vector<ed::MeasurementConstPtr>& measurements = it->second;
        for(std::vector<ed::MeasurementConstPtr>::const_iterator it2 = measurements.begin(); it2 != measurements.end(); ++it2)
        {
            const ed::MeasurementConstPtr& m = *it2;

            if (!rgb)
            {
                rgb = &m->image()->getRGBImage();
                canvas = 0.1 * rgb->clone();
            }

            for(ed::ImageMask::const_iterator it_p = m->imageMask().begin(canvas.cols); it_p != m->imageMask().end(); ++it_p)
            {
                canvas.at<cv::Vec3b>(*it_p) = rgb->at<cv::Vec3b>(*it_p);
                p_top_left.x = std::min(p_top_left.x, it_p->x);
                p_top_left.y = std::min(p_top_left.y, it_p->y);

                p_bottom_right.x = std::max(p_bottom_right.x, it_p->x);
                p_bottom_right.y = std::max(p_bottom_right.y, it_p->y);
            }
        }

        cv::Scalar color(0, 0, 200);

        cv::rectangle(canvas, p_top_left, p_bottom_right, color, 1);

        ed::EntityConstPtr e = world.getEntity(it->first);

        if (e)
        {
            std::string type = e->type();
            std::string info = e->id().str();

            if (info.size() > 4)
                info = info.substr(0, 4);

            // draw name background rectangle
            cv::rectangle(canvas, cv::Point(p_top_left.x, p_top_left.y) + cv::Point(0, -22),
                          cv::Point(p_top_left.x, p_top_left.y) + cv::Point(((type.size() + 6) * 10), -2),
                          color - cv::Scalar(140, 140, 140), CV_FILLED);

            // draw name and ID
            cv::putText(canvas, type + "(" + info + ")",
                        cv::Point(p_top_left.x, p_top_left.y) + cv::Point(5, -8),
                        1, 1.0, color, 1, CV_AA);
        }

    }

    if (canvas.data)
        pub.publish(canvas);
}

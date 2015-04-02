#include "visualization.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/measurement.h>

#include "ed_sensor_integration/properties/convex_hull.h"

#include <ed/error_context.h>

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

namespace
{
struct VisualizationLabel
{
    cv::Rect rect;
    std::string text;
    cv::Scalar color;
};

}

// ----------------------------------------------------------------------------------------------------

void visualizeUpdateRequest(const ed::WorldModel& world, const ed::UpdateRequest& req, ed::ImagePublisher& pub)
{
    ed::ErrorContext errc("Kinect 2 plugin: visualizeUpdateRequest");

    if (!pub.enabled())
        return;

    cv::Mat canvas;
    const cv::Mat* rgb = 0;

    std::vector<VisualizationLabel> labels;

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

            VisualizationLabel label;
            label.text = type + "(" + info + ")";
            label.rect = cv::Rect(p_top_left + cv::Point(0, -22), cv::Point(p_top_left.x, p_top_left.y) + cv::Point(((type.size() + 6) * 10)));
            label.color = color;

            labels.push_back(label);
        }

    }

    for(std::vector<VisualizationLabel>::iterator it = labels.begin(); it != labels.end(); ++it)
    {
        VisualizationLabel& label = *it;

        while(true)
        {
            const cv::Rect& r1 = label.rect;

            bool collides = false;

            for(std::vector<VisualizationLabel>::const_iterator it2 = labels.begin(); it2 != labels.end(); ++it2)
            {
                if (it2->text == label.text)
                    continue;

                const cv::Rect& r2 = it2->rect;
                if (r1.x + r1.width > r2.x && r2.x + r2.width > r1.x && r1.y + r1.height > r2.y && r2.y + r2.height > r1.y)
                {
                    collides = true;
                    break;
                }
            }

            if (!collides)
                break;

            label.rect.y -= label.rect.height;
        }

        // draw name background rectangle
        cv::rectangle(canvas, label.rect, label.color - cv::Scalar(140, 140, 140), CV_FILLED);

        // draw name and ID
        cv::putText(canvas, label.text, cv::Point(label.rect.x + 5, label.rect.y + 14),
                    1, 1.0, label.color, 1, CV_AA);

    }

    if (canvas.data)
        pub.publish(canvas);
}

#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <geolib/ros/tf_conversions.h>

#include <queue>

#include <dml/mesh_tools.h>
#include <geolib/Box.h>

// ----------------------------------------------------------------------------------------------------

class BeamCalculator
{

public:

    BeamCalculator(double w, unsigned int num_beams) : half_num_beams_(num_beams / 2)
    {
        fx_ = 2 * num_beams / w;

        rays_.resize(num_beams);
        for(unsigned int i = 0; i < num_beams; ++i)
            rays_[i] = geo::Vec2(((double)(i) - half_num_beams_) / fx_, 1);
    }

    inline int CalculateBeam(double x, double depth)
    {
        return (fx_ * x) / depth + half_num_beams_;
    }

    inline geo::Vec2 CalculatePoint(int i, double depth)
    {
        return rays_[i] * depth;
    }

    void RenderModel(const std::vector<geo::Vec2>& model, const geo::Transform2& pose, std::vector<double>& ranges)
    {
        std::vector<geo::Vec2> t_vertices(model.size());
        for(unsigned int i = 0; i < model.size(); ++i)
            t_vertices[i] = pose * model[i];

        int nbeams = num_beams();

        for(unsigned int i = 0; i < model.size(); ++i)
        {
            unsigned int j = (i + 1) % model.size();
            const geo::Vec2& p1 = t_vertices[i];
            const geo::Vec2& p2 = t_vertices[j];

            int i1 = CalculateBeam(p1.x, p1.y) + 1;
            int i2 = CalculateBeam(p2.x, p2.y);

            if (i2 < i1 || i2 < 0 || i1 >= nbeams)
                continue;

            i1 = std::max(0, i1);
            i2 = std::min(i2, nbeams - 1);

            geo::Vec2 s = p2 - p1;

            for(int i_beam = i1; i_beam <= i2; ++i_beam)
            {
                const geo::Vec2& r = rays_[i_beam];

                // calculate depth of intersection between line (p1, p2) and r
                double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);

                double& depth_old = ranges[i_beam];
                if (d > 0 && (d < depth_old || depth_old == 0))
                    depth_old = d;
            }
        }
    }

    inline unsigned int num_beams() const { return rays_.size(); }

    inline const std::vector<geo::Vec2>& rays() const { return rays_; }

private:

    double fx_;
    unsigned int half_num_beams_;
    std::vector<geo::Vec2> rays_;
};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    geo::Box box(geo::Vec3(-0.8, -0.4, 0), geo::Vec3(0.8, 0.4, 0.75));

    std::vector<std::vector<geo::Vec2> > contours;
    dml::project2D(box.getMesh(), contours);

    for(unsigned int i = 0; i < contours.size(); ++i)
    {
        std::cout << "Contour " << i << std::endl;
        const std::vector<geo::Vec2>& contour = contours[i];
        for(unsigned int j = 0; j < contour.size(); ++j)
            std::cout << "    " << contour[j] << std::endl;
        std::cout << std::endl;
    }

    if (contours.empty())
    {
        std::cout << "Could not create model contour" << std::endl;
        return 0;
    }

    // MODEL
    std::vector<geo::Vec2> model = contours.front();

    ros::init(argc, argv, "dml_test_poly");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    tf::TransformListener tf_listener;

    std::queue<rgbd::ImageConstPtr> image_buffer;

    BeamCalculator beam_calculator(2, 100);

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        // - - - - - - - - - - - - - - - - - -
        // Fetch kinect image and place in image buffer

        rgbd::ImageConstPtr rgbd_image = client.nextImage();
        if (rgbd_image && rgbd_image->getDepthImage().data)
            image_buffer.push(rgbd_image);

        if (image_buffer.empty())
            continue;

        rgbd_image = image_buffer.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute kinect pose based on TF

        geo::Pose3D sensor_pose;

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener.lookupTransform("/amigo/base_link", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
            geo::convert(t_sensor_pose, sensor_pose);
            image_buffer.pop();
        }
        catch(tf::ExtrapolationException& ex)
        {
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
                // to new, respectively). If it is too old, discard it.

                tf::StampedTransform latest_sensor_pose;
                tf_listener.lookupTransform("/amigo/base_link", rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
                // If image time stamp is older than latest transform, throw it out
                if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
                {
                    image_buffer.pop();
                    ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                    << ros::Time(rgbd_image->getTimestamp()));
                }

                continue;
            }
            catch(tf::TransformException& exc)
            {
                ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
                continue;
            }
        }
        catch(tf::TransformException& ex)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
            continue;
        }

        // Convert from ROS coordinate frame to geolib coordinate frame
        sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

        tf::Matrix3x3 m;
        geo::convert(sensor_pose.R, m);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        sensor_pose.R.setRPY(roll, pitch, 0);

//        std::cout << roll << " " << pitch << " " << yaw << std::endl;

        // - - - - - - - - - - - - - - - - - -

        cv::Mat depth = rgbd_image->getDepthImage();
        cv::Mat depth2 = depth.clone();

        rgbd::View view(*rgbd_image, depth.cols);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat reprojection(400, beam_calculator.num_beams(), CV_32FC1, 0.0);

        std::vector<float> ranges(beam_calculator.num_beams(), 0);

        for(int x = 0; x < depth.cols; ++x)
        {
            for(int y = 0; y < depth.rows; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d == 0 || d != d)
                    continue;

                geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
                geo::Vector3 p_floor = sensor_pose * p_sensor;

                if (p_floor.z < 0.2) // simple floor filter
                    continue;

                geo::Vec2 p_canvas(p_floor.x * 100 + canvas.cols / 2, canvas.rows - p_floor.y * 100);
                if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                {
                    int c = 10 + p_floor.z / 1.5 * 245;
//                    std::cout << p_floor.z << " " << c << std::endl;
                    cv::Vec3b& b = canvas.at<cv::Vec3b>(p_canvas.y, p_canvas.x);
                    if (b[0] < c)
                        b = cv::Vec3b(c, c, c);
                }

                int i = beam_calculator.CalculateBeam(p_floor.x, p_floor.y);
                if (i >= 0 && i < ranges.size())
                {
                    float& r = ranges[i];
                    if (r == 0 || p_floor.y < r)
                        r = p_floor.y;

                    cv::Point2i p_reprojection(i, (1.0 - p_floor.z / 2.0) * reprojection.rows);
                    if (p_reprojection.y >= 0 && p_reprojection.y < reprojection.rows)
                    {
                        float& d = reprojection.at<float>(p_reprojection);
                        if (d == 0 || p_floor.y < d)
                            d = p_floor.y;
                    }
                }

                if (i % 20 == 0)
                    depth2.at<float>(y, x) = depth.at<float>(y, x) + 1;
            }
        }

        for(unsigned int i = 0; i < ranges.size(); ++i)
        {
            geo::Vec2 p = beam_calculator.CalculatePoint(i, ranges[i]);
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 255, 0));
        }

        double min_error = 1e9;
        std::vector<double> best_model_ranges;
        geo::Transform2 best_pose;

        for(int i_beam = 0; i_beam < ranges.size(); ++i_beam)
        {
            double l = beam_calculator.rays()[i_beam].length();

            geo::Vec2 r = beam_calculator.rays()[i_beam] / l;

            for(double alpha = 0; alpha < 3.1415 * 2; alpha += 0.1)
            {
                double cos_alpha = cos(alpha);
                double sin_alpha = sin(alpha);
                geo::Mat2 rot(cos_alpha, -sin_alpha, sin_alpha, cos_alpha);

                geo::Transform2 pose(rot, r * 10);

                std::vector<double> model_ranges(ranges.size(), 0);
                beam_calculator.RenderModel(model, pose, model_ranges);

                double ds = ranges[i_beam];
                double dm = model_ranges[i_beam];

                if (ds <= 0 || dm <= 0)
                    continue;

                pose.t += r * ((ds - dm) * l);

                model_ranges.resize(ranges.size(), 0);
                beam_calculator.RenderModel(model, pose, model_ranges);

                int n = 0;
                double total_error = 0;
                for(unsigned int i = 0; i < model_ranges.size(); ++i)
                {
                    double ds = ranges[i];
                    double dm = model_ranges[i];

                    if (ds <= 0)
                        continue;

                    ++n;

                    if (dm <= 0)
                    {
                        total_error += 0.1;
                        continue;
                    }

                    double diff = std::abs(ds - dm);
                    if (diff < 0.1)
                        total_error += diff;
                    else
                    {
                        if (ds > dm)
                            total_error += 1;
                        else
                            total_error += 0.1;
                    }
                }

                double error = total_error / n;

                if (error < min_error)
                {
                    best_model_ranges = model_ranges;
                    best_pose = pose;
                    min_error = error;
                }
            }
        }

        for(unsigned int i = 0; i < best_model_ranges.size(); ++i)
        {
            geo::Vec2 p = beam_calculator.CalculatePoint(i, best_model_ranges[i]);
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 0, 255));
        }

        std::vector<geo::Vec2> t_vertices(model.size());
        for(unsigned int i = 0; i < model.size(); ++i)
            t_vertices[i] = best_pose * model[i];

        for(unsigned int i = 0; i < model.size(); ++i)
        {
            unsigned int j = (i + 1) % model.size();
            const geo::Vec2& p1 = t_vertices[i];
            const geo::Vec2& p2 = t_vertices[j];

            cv::Point p1_canvas(p1.x * 100 + canvas.cols / 2, canvas.rows - p1.y * 100);
            cv::Point p2_canvas(p2.x * 100 + canvas.cols / 2, canvas.rows - p2.y * 100);

            cv::line(canvas, p1_canvas, p2_canvas, cv::Scalar(0, 0, 255), 2);
        }



//        std::vector<geo::Vec2> points(ranges.size());
//        for(unsigned int i = 0; i < ranges.size(); ++i)
//            points[i] = geo::Vec2(view.getRasterizer().project2Dto3DX(i), 1) * ranges[i];

        // - - - - - - - - - - - - - - - - - -

        // Visualize
        cv::imshow("depth", depth2 / 10);
        cv::imshow("ranges", canvas);
        cv::imshow("reprojection", reprojection / 10);
        cv::waitKey(3);
    }

    return 0;
}

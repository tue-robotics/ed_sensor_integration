#include <ros/init.h>
#include <ros/node_handle.h>

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <ed/entity.h>
#include <geolib/Shape.h>
#include <rgbd/View.h>

// ----------------------------------------------------------------------------------------------------

class SimpleRenderResult : public geo::RenderResult
{

public:

    SimpleRenderResult(cv::Mat& z_buffer_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_) {}

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
        }
    }

    cv::Mat& z_buffer;

};

// ----------------------------------------------------------------------------------------------------

struct Snapshot
{
    rgbd::Image image;
    geo::Pose3D sensor_pose;
    std::string area_description;
};

// ----------------------------------------------------------------------------------------------------

bool readImage(const std::string& filename, rgbd::Image& image, geo::Pose3D& sensor_pose, std::string& area_description)
{
    std::ifstream f_in;
    f_in.open(filename.c_str());

    if (!f_in.is_open())
    {
        std::cerr << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << f_in.rdbuf();
    std::string json = buffer.str();

    ed::io::JSONReader r(json.c_str());

    std::string rgbd_filename;
    if (!r.readValue("rgbd_filename", rgbd_filename))
    {
        std::cerr << "No field 'rgbd_filename' specified." << std::endl;
        return false;
    }

    if (!r.readValue("area", area_description))
        area_description.clear();

    if (r.readGroup("sensor_pose"))
    {
        ed::deserialize(r, sensor_pose);
        r.endGroup();
    }
    else
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

    std::ifstream f_rgbd;
    f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

    if (!f_rgbd.is_open())
    {
        std::cerr << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    tue::serialization::InputArchive a_in(f_rgbd);
    rgbd::deserialize(a_in, image);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool loadWorldModel(const std::string& model_name, ed::WorldModel& world_model)
{
    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;

    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error))
    {
        std::cerr << "Model '" << model_name << "' could not be loaded:" << std::endl << std::endl;
        std::cerr << error.str() << std::endl;
        return false;
    }

    // Reset world
    world_model = ed::WorldModel();

    // Update world
    world_model.update(req);

    return true;
}

// ----------------------------------------------------------------------------------------------------

cv::Mat renderWorld(const ed::WorldModel& world, const geo::Pose3D& sensor_pose, const rgbd::Image& image)
{
    const cv::Mat& depth = image.getDepthImage();

    rgbd::View view(image, depth.cols);
    const geo::DepthCamera& cam_model = view.getRasterizer();

    cv::Mat depth_model(depth.rows, depth.cols, CV_32FC1, 0.0);
    SimpleRenderResult res(depth_model);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->shape() || !e->has_pose())
            continue;

        const geo::Mesh& mesh = e->shape()->getMesh();

        geo::Pose3D pose = sensor_pose.inverse() * e->pose();
        geo::RenderOptions opt;
        opt.setMesh(mesh, pose);

        // Render
        cam_model.render(opt, res);
    }

    return depth_model;
}

// ----------------------------------------------------------------------------------------------------

bool pointAssociates(float ds, float dm, const geo::DepthCamera& cam_model, int x, int y, float max_corr_dist_sq)
{
    if (dm != dm || dm == 0)
        return false;

    if (ds > dm)
        return true;

    float diff = ds - dm;
    if (diff * diff < max_corr_dist_sq)
        return true;

    return false;
}

// ----------------------------------------------------------------------------------------------------

cv::Mat calculateDifference(const cv::Mat& depth, const cv::Mat& depth_model, const geo::DepthCamera& cam_model, float max_corr_dist)
{
    cv::Mat difference = depth.clone();

    float max_corr_dist_sq = max_corr_dist * max_corr_dist;

    for(int y = 0; y < depth.rows; ++y)
    {
        for(int x = 0; x < depth.cols; ++x)
        {
            float ds = depth.at<float>(y, x);
            if (ds != ds || ds == 0)
                continue;

            if (ds > 2)
            {
                difference.at<float>(y, x) = 0;
                continue;
            }

            float dm = depth_model.at<float>(y, x);

            bool associates = pointAssociates(ds, dm, cam_model, x, y, max_corr_dist_sq);

            int w = max_corr_dist * cam_model.getOpticalCenterX() / ds;

            for(int d = 1; d < w && !associates; ++d)
            {
                associates =
                        (x - d >= 0 && pointAssociates(ds, dm, cam_model, x - d, y, max_corr_dist_sq)) ||
                        (x + d < depth.cols && pointAssociates(ds, dm, cam_model, x + d, y, max_corr_dist_sq)) ||
                        (y - d >= 0 && pointAssociates(ds, dm, cam_model, x, y - d, max_corr_dist_sq)) ||
                        (y + d < depth.rows && pointAssociates(ds, dm, cam_model, x, y + d, max_corr_dist_sq));

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

            if (associates)
            {
                difference.at<float>(y, x) = 0;
            }
        }
    }

    return difference;
}

// ----------------------------------------------------------------------------------------------------

void writeDepthImage(const std::string& filename, const cv::Mat& depth)
{
    cv::Mat rgb(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for(int y = 0; y < depth.rows; ++y)
    {
        for(int x = 0; x < depth.cols; ++x)
        {
            float d = depth.at<float>(y, x);
            if (d > 0 && d == d)
            {
                float d2 = std::min<float>(1.0, (d - 1) / 5);
                rgb.at<cv::Vec3b>(y, x) = d2 * cv::Vec3b(255, 255, 255);
            }
        }
    }

    cv::imwrite(filename, rgb);
}

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE WORLD-MODEL-NAME" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_segmenter");
    ros::NodeHandle nh;

    if (argc != 3)
    {
        usage();
        return 1;
    }

    std::string model_name = argv[2];

    ed::WorldModel world_model;
    if (!loadWorldModel(model_name, world_model))
        return 1;

    std::string image_filename = argv[1];

    Snapshot snapshot;
    if (!readImage(image_filename, snapshot.image, snapshot.sensor_pose, snapshot.area_description))
    {
        std::cerr << "Could not read " << image_filename << std::endl;
        return 1;
    }

    while(true)
    {

        const cv::Mat& depth = snapshot.image.getDepthImage();
        rgbd::View view(snapshot.image, depth.cols);
        const geo::DepthCamera& cam_model = view.getRasterizer();

        cv::Mat depth_model = renderWorld(world_model, snapshot.sensor_pose, snapshot.image);

        cv::Mat diff = calculateDifference(depth, depth_model, cam_model, 0.1);

        const cv::Mat& rgb = snapshot.image.getRGBImage();

        cv::Mat rgb_masked = rgb.clone();
        for(int y = 0; y < rgb.rows; ++y)
        {
            for(int x = 0; x < rgb.cols; ++x)
            {
                int x_depth = x * depth.cols / rgb.cols;
                int y_depth = y * depth.rows / rgb.rows;

                float d = diff.at<float>(y_depth, x_depth);
                if (d == 0 || d != d)
                {
                    rgb_masked.at<cv::Vec3b>(y, x) = 0.3 * rgb_masked.at<cv::Vec3b>(y, x);
                }
            }
        }

        for(int y = 1; y < depth.rows; ++y)
        {
            for(int x = 1; x < depth.cols; ++x)
            {
                float d = diff.at<float>(y, x);
                bool db = (d == d && d > 0);

                float d1 = diff.at<float>(y, x - 1);
                bool db1 = (d1 == d1 && d1 > 0);

                float d2 = diff.at<float>(y - 1, x);
                bool db2 = (d2 == d2 && d2 > 0);

                if (db != db1 || db != db2)
                {
                    int x_rgb = x * rgb.cols / depth.cols;
                    int y_rgb = y * rgb.rows / depth.rows;

                    cv::circle(rgb_masked, cv::Point(x_rgb, y_rgb), 2, cv::Scalar(0, 0, 255), CV_FILLED);
                }
            }
        }



        cv::imshow("depth", depth / 10);
        cv::imshow("depth model", depth_model / 10);
        cv::imshow("diff", diff / 10);
        cv::imshow("rgb masked", rgb_masked);
        char key = cv::waitKey();

        cv::imwrite("/tmp/cabinet-rgb.jpg", rgb);
        cv::imwrite("/tmp/cabinet-rgb-masked.jpg", rgb_masked);
        writeDepthImage("/tmp/cabinet-depth.png", depth);
        writeDepthImage("/tmp/cabinet-depth-model.png", depth_model);
        writeDepthImage("/tmp/cabinet-diff.png", diff);

        if (key == 81) // left
        {
            snapshot.sensor_pose.t.x -= 0.05;
        }
        else if (key == 82) // up
        {
            snapshot.sensor_pose.t.z  += 0.05;
        }
        else if (key == 83) // right
        {
            snapshot.sensor_pose.t.x += 0.05;
        }
        else if (key == 84) // down
        {
            snapshot.sensor_pose.t.z -= 0.05;
        }
        else if (key == 'w') // w
        {
            snapshot.sensor_pose.t.y += 0.05;
        }
        else if (key == 's') // s
        {
            snapshot.sensor_pose.t.y -= 0.05;
        }


        std::cout << (int)key << std::endl;
        if (key == 'q')
            break;
    }

    return 0;
}

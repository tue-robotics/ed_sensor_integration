#include <ros/init.h>
#include <ros/node_handle.h>

#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>
#include <ed/kinect/updater.h>
#include <ed/kinect/fitter.h>

#include <tue/filesystem/crawler.h>

#include <rgbd/image.h>
#include <rgbd/serialization.h>
#include <rgbd/view.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/data_pointer.h>

#include <ed/entity.h>

#include "geolib/math_types.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <vector>

// ----------------------------------------------------------------------------------------------------

struct Snapshot
{
    rgbd::ImagePtr image;
    geo::Pose3D sensor_pose;
    std::string area_description;
    ed::WorldModel world_model;
};

// ----------------------------------------------------------------------------------------------------

bool readImage(const std::string& filename, rgbd::ImagePtr& image, geo::Pose3D& sensor_pose,
               ed::WorldModel& world_model, std::string& area_description)
{
    tue::config::DataPointer meta_data;

    try
    {
        meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;
        return false;
    }

    tue::config::Reader r(meta_data);

    // Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

        std::ifstream f_rgbd;
        f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

        if (!f_rgbd.is_open())
        {
            std::cerr << "Could not open '" << filename << "'." << std::endl;
            return false;
        }

        image.reset(new rgbd::Image);

        tue::serialization::InputArchive a_in(f_rgbd);
        rgbd::deserialize(a_in, *image);
    }

    // Read sensor pose
    if (!ed::deserialize(r, "sensor_pose", sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

//    if (r.hasError())
//    {
//        std::cout << "Error while reading file '" << filename << "':\n\n" << r.error() << std::endl;
//        return false;
//    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool loadWorldModel(const std::string& model_name, ed::WorldModel& world_model)
{
    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;

    std::stringstream error;
    if (!model_loader.create("_root", model_name, req, error, true))
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

void usage()
{
    std::cout << "Usage: ed_segmenter IMAGE-FILE-OR-DIRECTORY  WORLDMODEL_NAME" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

/*
void showSegmentationResults(const Snapshot& snapshot, const UpdateRequest& update_req, const UpdateResult& res, cv::Mat& canvas) {
    std::cout << update_req.measurements.size() << std::endl;

    std::string error_msg = res.error.str();
    if (error_msg.empty())
    {
        int depth_width = snapshot->image->getDepthImage().cols;
        double f = (double)canvas->cols / depth_width;

        for(unsigned int i = 0; i < res->entity_updates.size(); ++i)
        {
            const EntityUpdate& e_update = res.entity_updates[i];
            if (e_update.pixel_indices.empty())
                continue;

            unsigned i_pxl = e_update.pixel_indices[0];
            cv::Point bb_min(i_pxl % depth_width, i_pxl / depth_width);
            cv::Point bb_max(i_pxl % depth_width, i_pxl / depth_width);

            for(std::vector<unsigned int>::const_iterator it = e_update.pixel_indices.begin(); it != e_update.pixel_indices.end(); ++it)
            {
                int x = *it % depth_width;
                int y = *it / depth_width;

                bb_min.x = std::min(bb_min.x, x);
                bb_min.y = std::min(bb_min.y, y);
                bb_max.x = std::max(bb_max.x, x);
                bb_max.y = std::max(bb_max.y, y);

                for(double x2 = f * x; x2 < (f * (x + 1)); ++x2)
                    for(double y2 = f * y; y2 < (f * (y + 1)); ++y2)
                        canvas->at<cv::Vec3b>(y2, x2) = cv::Vec3b(0, 0, 255);
            }

            cv::Point d(2, 2);
            cv::rectangle(canvas, f * bb_min, f * bb_max, cv::Scalar(255, 255, 255), 2);
            cv::rectangle(canvas, f * bb_min - d, f * bb_max + d, cv::Scalar(0, 0, 0), 2);
            cv::rectangle(canvas, f * bb_min + d, f * bb_max - d, cv::Scalar(0, 0, 0), 2);
        }

    }
    else
    {
        std::cerr << error_msg << std::endl;
        cv::putText(canvas, "Segmentation failed", cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);
    }
}

*/
// ----------------------------------------------------------------------------------------------------

// Getting roll, pitch and yaw from a quaternion,
// copied from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

double getYaw(const geo::Mat3& rotation)
{
    // Get quaternion
    geo::Quaternion geo_quaternion;
    rotation.getRotation(geo_quaternion);

    // Convert it to struct
    Quaternion quaternion;
    quaternion.x = geo_quaternion.getX();
    quaternion.y = geo_quaternion.getY();
    quaternion.z = geo_quaternion.getZ();
    quaternion.w = geo_quaternion.getW();

    // Get the Euler angles
    EulerAngles angles = ToEulerAngles(quaternion);
    ROS_DEBUG_STREAM("Matrix: " << rotation << " --> yaw: " << angles.yaw);
    return angles.yaw;
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

    tue::filesystem::Path path = argv[1];
    if (!path.exists())
    {
        std::cerr << "Path '" << path << "' does not exist." << std::endl;
        return 1;
    }

    tue::filesystem::Crawler crawler;

    if (path.isDirectory())
        crawler.setRootPath(path);
    else
        crawler.setRootPath(path.parentPath());

    Updater updater;
    Fitter fitter;

    std::vector<Snapshot> snapshots;
    unsigned int i_snapshot = 0;

    while(ros::ok())
    {
        if (i_snapshot >= snapshots.size())
        {
            bool file_found = true;
            tue::filesystem::Path filename;

            if (path.isRegularFile() && snapshots.empty())
                filename = path;
            else
            {
                file_found = false;
                while (crawler.nextPath(filename))
                {
                    if (filename.extension() == ".json")
                    {
                        file_found = true;
                        break;
                    }
                }
            }

            if (file_found)
            {
                i_snapshot = snapshots.size();
                snapshots.push_back(Snapshot());
                Snapshot& snapshot = snapshots.back();
                snapshot.world_model = world_model;

                if (!readImage(filename.string(), snapshot.image, snapshot.sensor_pose,
                               snapshot.world_model, snapshot.area_description))
                {
                    std::cerr << "Could not read " << filename << std::endl;
                    snapshots.pop_back();
                    continue;
                }
                else
                {
//                    std::cout << "Successfully loaded " << filename << std::endl;
                }
            }
            else
            {
                if (snapshots.empty())
                    break;

                i_snapshot = snapshots.size() - 1;
            }
        }

        Snapshot& snapshot = snapshots[i_snapshot];

        //ed::UpdateRequest update_req;
        //UpdateResult res(update_req);

        //UpdateRequest kinect_update_request;
        //kinect_update_request.area_description = "on_top_of dinner_table";
        //updater.update(snapshot.world_model, snapshot.image, snapshot.sensor_pose, kinect_update_request, res);

        FitterData data;
        geo::Pose3D fitted_pose;

        ed::EntityConstPtr e = snapshot.world_model.getEntity("dinner_table");

        fitter.processSensorData(*snapshot.image, snapshot.sensor_pose, data);

        fitter.estimateEntityPose(data, snapshot.world_model, "dinner_table", e->pose(), fitted_pose);

        // show snapshot
        cv::Mat rgbcanvas = snapshot.image->getRGBImage().clone();
        cv::imshow("RGB", rgbcanvas);

        // visualise fitting (positive y direction = downwards)
        int canvas_width = 600;
        int canvas_height = 600;
        cv::Mat canvas = cv::Mat(canvas_height, canvas_width, CV_8UC3, cv::Scalar(100,100,100));

        // needed parameters: Fitterdata data;
        int sensor_x = canvas_width/2;
        int sensor_y = canvas_height * 8/10;
        float canvas_resolution = 100; //pixels per meter

        // paint to screen the location of HERO
        cv::Point sensorlocation(sensor_x, sensor_y);
        cv::Scalar sensorcolor(0,255,0); // green
        cv::circle(canvas, sensorlocation, 3, sensorcolor, cv::FILLED);

        // paint sensor_ranges
        for(unsigned int i = 0; i < data.sensor_ranges.size(); ++i){
            float fx_ = 2 * data.sensor_ranges.size() / 4;
            float a = (((float) i - (data.sensor_ranges.size()/2)) / fx_); // TODO: remove hard-coded values, both fx_ and amount of sensor_ranges is hardcoded
            float x_m = data.sensor_ranges[i] * sin(a);
            float y_m = data.sensor_ranges[i] * cos(a);

            // postion to pixels
            int x_p = sensor_x + (int)(x_m * canvas_resolution);
            int y_p = sensor_y - (int)(y_m * canvas_resolution);

            if (x_p < 0 || x_p >= canvas_width)
                continue;
            if (y_p < 0 || y_p >= canvas_height)
                continue;
            if (data.sensor_ranges[i] == 0) // filter out sensor_ranges equal to zero
                continue;

            // paint to screen
            cv::Point centerCircle(x_p, y_p);
            cv::Scalar colorCircle(0,0,255);
            cv::circle(canvas, centerCircle, 2, colorCircle, cv::FILLED);
        }

    // paint entity (from worldmodel)
        EntityRepresentation2D entity_2d = fitter.GetOrCreateEntity2D(e);

        geo::Pose3D pose = e->pose();

        // worldmodel entity yaw (unfitted)
        float yaw_ent = getYaw(pose.R);

        // robot yaw
        float yaw_robot = getYaw(snapshot.sensor_pose.R);

        for (int i=0; i < entity_2d.shape_2d.size(); i++){

            for (int j=0; j < entity_2d.shape_2d[i].size()-1; j++){
                // computing outer corner positions (in worldmodel coordinate system)
                float x_wm1 = e->pose().t.x + (cos(yaw_ent) * entity_2d.shape_2d[i][j].x + sin(yaw_ent) * entity_2d.shape_2d[i][j].y);
                float y_wm1 = e->pose().t.y + (-sin(yaw_ent) * entity_2d.shape_2d[i][j].x + cos(yaw_ent) * entity_2d.shape_2d[i][j].y);
                float x_wm2 = e->pose().t.x + (cos(yaw_ent) * entity_2d.shape_2d[i][j+1].x + sin(yaw_ent) * entity_2d.shape_2d[i][j+1].y);
                float y_wm2 = e->pose().t.y + (-sin(yaw_ent) * entity_2d.shape_2d[i][j+1].x + cos(yaw_ent) * entity_2d.shape_2d[i][j+1].y);

                // computing vector from robot to entity corners (in worldmodel coordinate system)
                float diff_x1 = x_wm1 - snapshot.sensor_pose.t.x;
                float diff_y1 = y_wm1 - snapshot.sensor_pose.t.y;
                float diff_x2 = x_wm2 - snapshot.sensor_pose.t.x;
                float diff_y2 = y_wm2 - snapshot.sensor_pose.t.y;

                // rotating coordinate system to that of the robot
                float x_m1 = cos(yaw_robot) * diff_x1 + sin(yaw_robot) * diff_y1;
                float y_m1 = -sin(yaw_robot) * diff_x1 + cos(yaw_robot) * diff_y1;
                float x_m2 = cos(yaw_robot) * diff_x2 + sin(yaw_robot) * diff_y2;
                float y_m2 = -sin(yaw_robot) * diff_x2 + cos(yaw_robot) * diff_y2;

                // position to pixels
                int x_p1 = sensor_x + (int)(x_m1 * canvas_resolution);
                int y_p1 = sensor_y - (int)(y_m1 * canvas_resolution);
                int x_p2 = sensor_x + (int)(x_m2 * canvas_resolution);
                int y_p2 = sensor_y - (int)(y_m2 * canvas_resolution);

                if (x_p1 < 0 || x_p2 < 0|| x_p1 >= canvas_width || x_p2 >= canvas_width){
                    std::cout << "Entity: x-coordinate out of range" << std::endl;
                    continue;
                }
                if (y_p1 < 0 || y_p2 < 0|| y_p1 >= canvas_height || y_p2 >= canvas_height){
                    std::cout << "Entity: y-coordinate out of range" << std::endl;
                    continue;
                }

                // paint to screen
                cv::Point point1(x_p1, y_p1);
                cv::Point point2(x_p2, y_p2);
                cv::Scalar colorLine(0,255,255); //Yellow, B G R
                cv::line(canvas, point1, point2, colorLine, 1);
            }
        }

        // adding last edge of entity (begin point to end point) (in worldmodel coordinate system)
        float x_wm_start = e->pose().t.x + (cos(yaw_ent) * entity_2d.shape_2d[0][0].x + sin(yaw_ent) * entity_2d.shape_2d[0][0].y);
        float y_wm_start = e->pose().t.y + (-sin(yaw_ent) * entity_2d.shape_2d[0][0].x + cos(yaw_ent) * entity_2d.shape_2d[0][0].y);
        float x_wm_end = e->pose().t.x + (cos(yaw_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].x + sin(yaw_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].y);
        float y_wm_end = e->pose().t.y + (-sin(yaw_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].x + cos(yaw_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].y);

        //computing vector from robot to entity corners (in worldmodel coordinate system)
        float diff_x_start = x_wm_start - snapshot.sensor_pose.t.x;
        float diff_y_start = y_wm_start - snapshot.sensor_pose.t.y;
        float diff_x_end = x_wm_end - snapshot.sensor_pose.t.x;
        float diff_y_end = y_wm_end - snapshot.sensor_pose.t.y;

        // rotating coordinate system to that of the robot
        float x_m_start = cos(yaw_robot) * diff_x_start + sin(yaw_robot) * diff_y_start;
        float y_m_start = -sin(yaw_robot) * diff_x_start + cos(yaw_robot) * diff_y_start;
        float x_m_end = cos(yaw_robot) * diff_x_end + sin(yaw_robot) * diff_y_end;
        float y_m_end = -sin(yaw_robot) * diff_x_end + cos(yaw_robot) * diff_y_end;

        // position to pixels
        int x_p_start = sensor_x + (int)(x_m_start * canvas_resolution);
        int y_p_start = sensor_y - (int)(y_m_start * canvas_resolution);
        int x_p_end = sensor_x + (int)(x_m_end * canvas_resolution);
        int y_p_end = sensor_y - (int)(y_m_end * canvas_resolution);

        // paint last edge to screen
        cv::Point point_begin(x_p_start, y_p_start);
        cv::Point point_end(x_p_end, y_p_end);
        cv::Scalar colorLine(0,255,255);
        cv::line(canvas, point_begin, point_end, colorLine, 1);

    // paint fitted entity
        float yaw_fit_ent = getYaw(fitted_pose.R);

        for (int i=0; i < entity_2d.shape_2d.size(); i++){

            for (int j=0; j < entity_2d.shape_2d[i].size()-1; j++){
                // computing outer edge positions (in worldmodel coordinate system)
                float x_wm_f_1 = fitted_pose.t.x + (cos(yaw_fit_ent) * entity_2d.shape_2d[i][j].x + sin(yaw_fit_ent) * entity_2d.shape_2d[i][j].y);
                float y_wm_f_1 = fitted_pose.t.y + (-sin(yaw_fit_ent) * entity_2d.shape_2d[i][j].x + cos(yaw_fit_ent) * entity_2d.shape_2d[i][j].y);
                float x_wm_f_2 = fitted_pose.t.x + (cos(yaw_fit_ent) * entity_2d.shape_2d[i][j+1].x + sin(yaw_fit_ent) * entity_2d.shape_2d[i][j+1].y);
                float y_wm_f_2 = fitted_pose.t.y + (-sin(yaw_fit_ent) * entity_2d.shape_2d[i][j+1].x + cos(yaw_fit_ent) * entity_2d.shape_2d[i][j+1].y);

                // computing vector from robot to entity corners (in worldmodel coordinate system)
                float diff_x_f_1 = x_wm_f_1 - snapshot.sensor_pose.t.x;
                float diff_y_f_1 = y_wm_f_1 - snapshot.sensor_pose.t.y;
                float diff_x_f_2 = x_wm_f_2 - snapshot.sensor_pose.t.x;
                float diff_y_f_2 = y_wm_f_2 - snapshot.sensor_pose.t.y;

                // rotating coordinate system to that of the robot
                float x_m_f_1 = cos(yaw_robot) * diff_x_f_1 + sin(yaw_robot) * diff_y_f_1;
                float y_m_f_1 = -sin(yaw_robot) * diff_x_f_1 + cos(yaw_robot) * diff_y_f_1;
                float x_m_f_2 = cos(yaw_robot) * diff_x_f_2 + sin(yaw_robot) * diff_y_f_2;
                float y_m_f_2 = -sin(yaw_robot) * diff_x_f_2 + cos(yaw_robot) * diff_y_f_2;

                // position to pixels
                int x_p_f_1 = sensor_x + (int)(x_m_f_1 * canvas_resolution);
                int y_p_f_1 = sensor_y - (int)(y_m_f_1 * canvas_resolution);
                int x_p_f_2 = sensor_x + (int)(x_m_f_2 * canvas_resolution);
                int y_p_f_2 = sensor_y - (int)(y_m_f_2 * canvas_resolution);

                if (x_p_f_1 < 0 || x_p_f_2 < 0|| x_p_f_1 >= canvas_width || x_p_f_2 >= canvas_width){
                    std::cout << "Fitted entity: x-coordinate out of range" << std::endl;
                    continue;
                }
                if (y_p_f_1 < 0 || y_p_f_2 < 0|| y_p_f_1 >= canvas_height || y_p_f_2 >= canvas_height){
                    std::cout << "Fitted entity: y-coordinate out of range" << std::endl;
                    continue;
                }

                // paint to screen
                cv::Point point1(x_p_f_1, y_p_f_1);
                cv::Point point2(x_p_f_2, y_p_f_2);
                cv::Scalar colorLine2(255,0,0); //blue
                cv::line(canvas, point1, point2, colorLine2, 1);
            }
        }

        // adding last edge of entity (begin point to end point)
        float x_wm_f_start = fitted_pose.t.x + (cos(yaw_fit_ent) * entity_2d.shape_2d[0][0].x + sin(yaw_fit_ent) * entity_2d.shape_2d[0][0].y);
        float y_wm_f_start = fitted_pose.t.y + (-sin(yaw_fit_ent) * entity_2d.shape_2d[0][0].x + cos(yaw_fit_ent) * entity_2d.shape_2d[0][0].y);
        float x_wm_f_end = fitted_pose.t.x + (cos(yaw_fit_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].x + sin(yaw_fit_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].y);
        float y_wm_f_end = fitted_pose.t.y + (-sin(yaw_fit_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].x + cos(yaw_fit_ent) * entity_2d.shape_2d[0][entity_2d.shape_2d[0].size()-1].y);

        // computing vector from robot to entity corners (in worldmodel coordinate system)
        float diff_x_f_start = x_wm_f_start - snapshot.sensor_pose.t.x;
        float diff_y_f_start = y_wm_f_start - snapshot.sensor_pose.t.y;
        float diff_x_f_end = x_wm_f_end - snapshot.sensor_pose.t.x;
        float diff_y_f_end = y_wm_f_end - snapshot.sensor_pose.t.y;

        // rotating coordinate system to that of the robot
        float x_m_f_start = cos(yaw_robot) * diff_x_f_start + sin(yaw_robot) * diff_y_f_start;
        float y_m_f_start = -sin(yaw_robot) * diff_x_f_start + cos(yaw_robot) * diff_y_f_start;
        float x_m_f_end = cos(yaw_robot) * diff_x_f_end + sin(yaw_robot) * diff_y_f_end;
        float y_m_f_end = -sin(yaw_robot) * diff_x_f_end + cos(yaw_robot) * diff_y_f_end;

        // position to pixels
        int x_p_f_start = sensor_x + (int)(x_m_f_start * canvas_resolution);
        int y_p_f_start = sensor_y - (int)(y_m_f_start * canvas_resolution);
        int x_p_f_end = sensor_x + (int)(x_m_f_end * canvas_resolution);
        int y_p_f_end = sensor_y - (int)(y_m_f_end * canvas_resolution);

        //std::cout << "de begin coordinaat is: ("<< x_p_start << "),("<< y_p_start << ")" << std::endl;
        //std::cout << "de eind coordinaat is: ("<< x_p_end << "),("<< y_p_end << ")" << std::endl;
        std::cout << "Robot pose: ("<< snapshot.sensor_pose << ")" << std::endl;
        std::cout << "Worldmodel object pose: ("<< e->pose() << ")" << std::endl;
        std::cout << "Fitted object pose: ("<< fitted_pose << ")" << std::endl;
        std::cout << "Yaw of robot: ("<< yaw_robot << ")" << std::endl;
        std::cout << "Yaw of worldmodel object pose: ("<< yaw_ent << ")" << std::endl;
        std::cout << "Yaw of fitted object pose: ("<< yaw_fit_ent << ")" << std::endl;
        std::cout << data.sensor_ranges.size() << std::endl;
        //std::cout << valid_sensor_ranges.size() << std::endl;


        // paint last edge to screen
        cv::Point point_f_start(x_p_f_start, y_p_f_start);
        cv::Point point_f_end(x_p_f_end, y_p_f_end);
        cv::Scalar colorLine2(255,0,0);
        cv::line(canvas, point_f_start, point_f_end, colorLine2, 1);

        cv::imshow("Fitting", canvas);
        char key = cv::waitKey();

//        if (key == 81)  // Left arrow
//        {
//            if (i_snapshot > 0)
//                --i_snapshot;
//        }
//        else if (key == 83) // Right arrow
//        {
//            ++i_snapshot;
//        }
//        else if (key == 'q')
//        {
//            break;
//        }

//        //        std::cout << (int)key << std::endl;
    }

    return 0;
}

#include "ed_sensor_integration/FitModel.h"
#include "ed_sensor_integration/GetModels.h"
#include "ed_sensor_integration/GetSnapshots.h"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ed_sensor_integration/GUIAction.h"

// ----------------------------------------------------------------------------------------------------

double BUTTON_SIZE = 30;
double MODEL_SIZE = 180;
double BORDER_SIZE = 10;

// ----------------------------------------------------------------------------------------------------

struct Model
{
    std::string name;
    cv::Mat image;
};

// ----------------------------------------------------------------------------------------------------

struct View
{
    View() {}
    View(cv::Mat& img_, const cv::Rect& rect_) : rect(rect_), roi(img_(rect)) {}

    cv::Rect rect;
    cv::Mat roi;
};

// ----------------------------------------------------------------------------------------------------

unsigned int revision_ = 0;
std::map<std::string, cv::Mat> snapshots_;
ros::ServiceClient cl_fit_model_;
std::string current_image_id_;

// Models
bool models_requested_ = false;
std::vector<Model> models_;
unsigned int i_model_start = 0;
int i_model_selected = -1;

View snapshot_view;
View model_view;

// GUI actions
ed_sensor_integration::GUIAction::ConstPtr gui_action_;

void DrawModels(cv::Mat& img, unsigned int i_start);

// ----------------------------------------------------------------------------------------------------

void mouseCallback(int event, int x, int y, int flags, void* ptr)
{
    cv::Point mpos(x, y);

    if (event == CV_EVENT_LBUTTONDOWN)
    {
        if (model_view.rect.contains(mpos))
        {
            mpos -= model_view.rect.tl();

            if (mpos.y < BUTTON_SIZE)
            {
                if (i_model_start > 0)
                    --i_model_start;
            }
            else if (mpos.y > model_view.rect.height - BUTTON_SIZE)
            {
                if (i_model_start < models_.size() - 1)
                    ++i_model_start;
            }
            else
            {
                i_model_selected = i_model_start + (mpos.y - BUTTON_SIZE) / (MODEL_SIZE + BORDER_SIZE);
            }

            DrawModels(model_view.roi, i_model_start);
        }
        else if (snapshot_view.rect.contains(mpos))
        {
            std::cout << "SNAPSHOT" << std::endl;
        }


//        if (current_image_id_.empty())
//        {
//            std::cout << "No image" << std::endl;
//            return;
//        }

//        ed_sensor_integration::FitModel srv;
//        srv.request.click_x = x;
//        srv.request.click_y = y;
//        srv.request.image_id = current_image_id_;
//        srv.request.model_name = "robotics_testlab_B.hallway_cabinet";

//        if (cl_fit_model_.call(srv))
//        {

//        }
    }
}

// ----------------------------------------------------------------------------------------------------

void DrawModels(cv::Mat& img, unsigned int i_start)
{
    img.setTo(cv::Vec3b(20, 20, 20));

    int y = BUTTON_SIZE + BORDER_SIZE;
    double width = img.cols;
    for(unsigned int i = i_start; i < models_.size(); ++i)
    {       
        std::cout << i << ": " << y << std::endl;

        const cv::Mat& model_img = models_[i].image;
        double ratio = std::min((width - 2 * BORDER_SIZE) / model_img.cols,
                                MODEL_SIZE / model_img.cols);

        if (y + MODEL_SIZE > img.rows - BUTTON_SIZE - BORDER_SIZE)
        {
            std::cout << y + MODEL_SIZE << std::endl;
            break;
        }

        cv::Rect rect(cv::Point(BORDER_SIZE, y), cv::Size(ratio * model_img.cols, ratio * model_img.rows));
        cv::Mat roi = img(rect);

        cv::resize(model_img, roi, rect.size());

        if (i == i_model_selected)
        {
            cv::rectangle(img, rect, cv::Scalar(50, 50, 255), 3);
        }

        y += MODEL_SIZE + BORDER_SIZE;
    }

    cv::rectangle(img, cv::Point(5, 0), cv::Point(width - 5, BUTTON_SIZE), cv::Scalar(20, 20, 100), 2, 1);
    cv::rectangle(img, cv::Point(5, img.rows - BUTTON_SIZE), cv::Point(width - 5, img.rows), cv::Scalar(20, 20, 100), 2, 1);

}

// ----------------------------------------------------------------------------------------------------

void DrawSnapshot(cv::Mat& img)
{
    cv::rectangle(img, cv::Point(0, 0), cv::Point(BUTTON_SIZE, img.rows), cv::Scalar(20, 20, 100), 2, 1);
    cv::rectangle(img, cv::Point(img.cols - BUTTON_SIZE, 0), cv::Point(img.cols, img.rows), cv::Scalar(20, 20, 100), 2, 1);

    std::map<std::string, cv::Mat>::const_iterator it = snapshots_.find(current_image_id_);
    if (it == snapshots_.end())
        return;

    const cv::Mat& snapshot = it->second;

    double ratio = std::min((double)(img.cols - 2 * (BUTTON_SIZE + BORDER_SIZE)) / snapshot.cols,
                            (double)(img.rows - 2 * BORDER_SIZE) / snapshot.rows);

    cv::Rect rect(cv::Point(BUTTON_SIZE + BORDER_SIZE, BORDER_SIZE),
                  cv::Size(ratio * snapshot.cols, ratio * snapshot.rows));

    cv::Mat roi = img(rect);
    cv::resize(snapshot, roi, rect.size());
}

// ----------------------------------------------------------------------------------------------------

void DrawAction()
{

}

// ----------------------------------------------------------------------------------------------------

void cbGUIAction(const ed_sensor_integration::GUIAction::ConstPtr& msg)
{
    if (msg->action == "switch_snapshot")
    {
        current_image_id_ = msg->params[0];
        DrawSnapshot(snapshot_view.roi);
    }
    else if (msg->action == "fit_model")
    {
        std::string model_name = msg->params[0];
        std::string image_id = msg->params[1];
        double click_x_ratio = atof(msg->params[2].c_str());
        double click_y_ratio = atof(msg->params[3].c_str());
    }

    gui_action_ = msg;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_fitter_gui");

    ros::NodeHandle nh;

    ros::ServiceClient cl_get_snapshots = nh.serviceClient<ed_sensor_integration::GetSnapshots>("ed/gui/get_snapshots");
    ros::ServiceClient cl_get_models = nh.serviceClient<ed_sensor_integration::GetModels>("ed/gui/get_models");
    cl_fit_model_ = nh.serviceClient<ed_sensor_integration::FitModel>("ed/gui/fit_model");

    ros::Subscriber sub_gui_actions_= nh.subscribe<ed_sensor_integration::GUIAction>("ed/gui/viz_action", 1, cbGUIAction);

    cv::Mat canvas(480, 840, CV_8UC3, cv::Scalar(20, 20, 20));

    cv::namedWindow("Fitter GUI", 1);
    cv::setMouseCallback("Fitter GUI", mouseCallback);

    snapshot_view = View(canvas, cv::Rect(cv::Point(0, 0), cv::Size(640, 480)));
    model_view = View(canvas, cv::Rect(cv::Point(640, 0), cv::Size(200, 480)));

    ros::Time t_last_update(0);

    while(ros::ok())
    {
        ros::Time time = ros::Time::now();

        if (time - t_last_update > ros::Duration(1.0))
        {
            if (!models_requested_)
            {
                ed_sensor_integration::GetModels srv;
                if (cl_get_models.call(srv))
                {
                    std::cout << "received: " << srv.response.model_images.size() << " models" << std::endl;

                    models_.resize(srv.response.model_names.size());
                    for(unsigned int i = 0; i < srv.response.model_names.size(); ++i)
                    {
                        Model& m = models_[i];
                        m.image = cv::imdecode(srv.response.model_images[i].data, CV_LOAD_IMAGE_UNCHANGED);
                        m.name = srv.response.model_names[i];

                    }

                    DrawModels(model_view.roi, 0);

                    models_requested_ = true;
                }
                else
                {
                    std::cout << "Could not query models" << std::endl;
                }
            }

            ed_sensor_integration::GetSnapshots srv;
            srv.request.revision = revision_;

            if (cl_get_snapshots.call(srv))
            {
                std::cout << "received: " << srv.response.images.size() << " new images" << std::endl;

                for(unsigned int i = 0; i < srv.response.images.size(); ++i)
                {
                    cv::Mat image = cv::imdecode(srv.response.images[i].data, CV_LOAD_IMAGE_UNCHANGED);
                    snapshots_[srv.response.image_ids[i]] = image;
                    current_image_id_ = srv.response.image_ids[i];
                }
                revision_ = srv.response.new_revision;

                DrawSnapshot(snapshot_view.roi);
            }
            else
            {
                ROS_ERROR("Could not call service");
            }

            t_last_update = time;
        }

        cv::imshow("Fitter GUI", canvas);
        cv::waitKey(30);
    }

    return 0;
}

#include <ros/ros.h>

#include <ed_sensor_integration/Segment.h>

#include <ed/uuid.h>
#include <ed_gui_server/GetEntityInfo.h>
#include <ed_perception/Classify.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

bool fullColumnHasColor(const cv::Mat& img, int x, const cv::Vec3b& clr)
{
    for(int y = 0; y < img.rows; ++y)
    {
        if (img.at<cv::Vec3b>(y, x) != clr)
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool fullRowHasColor(const cv::Mat& img, int y, const cv::Vec3b& clr)
{
    for(int x = 0; x < img.cols; ++x)
    {
        if (img.at<cv::Vec3b>(y, x) != clr)
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

cv::Rect getBoundingRect(const cv::Mat& img)
{
    int x_min = 0;
    for(int x = 0; x < img.cols; ++x)
    {
        if (!fullColumnHasColor(img, x, cv::Vec3b(0, 0, 0)))
        {
            x_min = x;
            break;
        }
    }

    int x_max = img.cols - 1;
    for(int x = x_max; x >= 0; --x)
    {
        if (!fullColumnHasColor(img, x, cv::Vec3b(0, 0, 0)))
        {
            x_max = x;
            break;
        }
    }

    int y_min = 0;
    for(int y = 0; y < img.rows; ++y)
    {
        if (!fullRowHasColor(img, y, cv::Vec3b(0, 0, 0)))
        {
            y_min = y;
            break;
        }
    }

    int y_max = img.rows - 1;
    for(int y = y_max; y >= 0; --y)
    {
        if (!fullRowHasColor(img, y, cv::Vec3b(0, 0, 0)))
        {
            y_max = y;
            break;
        }
    }

    return cv::Rect(cv::Point(x_min, y_min), cv::Point(x_max, y_max));
}

// ----------------------------------------------------------------------------------------------------

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << x << std::endl;
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //          std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        mouse_pos = cv::Vec2i(x, y);
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segment_and_show");
    ros::NodeHandle nh;
    ros::ServiceClient cl_segment = nh.serviceClient<ed_sensor_integration::Segment>("ed/kinect/segment");
    ros::ServiceClient cl_info = nh.serviceClient<ed_gui_server::GetEntityInfo>("ed/gui/get_entity_info");
    ros::ServiceClient cl_perception = nh.serviceClient<ed_perception::Classify>("ed/classify");

    ed_sensor_integration::Segment srv;
    srv.request.max_sensor_range = 2.0;

    if (!cl_segment.call(srv))
    {
        std::cout << "Could not call segment service." << std::endl;
        return 1;
    }

    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(255, 0,   0));
    colors.push_back(cv::Scalar(0,   255, 0));
    colors.push_back(cv::Scalar(0,   0,   255));
    colors.push_back(cv::Scalar(255, 255,   0));
    colors.push_back(cv::Scalar(0,   255, 255));
    colors.push_back(cv::Scalar(255,   0,   255));


    int i_color = 0;
    cv::Mat img_combined;

    std::vector<std::string> ids;
    std::vector<cv::Rect> rois;

    for(std::vector<std::string>::const_iterator it = srv.response.entity_ids.begin(); it != srv.response.entity_ids.end(); ++it)
    {
        ed::UUID id = *it;

        ed_gui_server::GetEntityInfo srv;
        srv.request.id = id.str();
        srv.request.measurement_image_border = 20;

        if (!cl_info.call(srv))
        {
            std::cout << "Could not call info service (for measurement image)" << std::endl;
            continue;
        }

        cv::Mat img = cv::imdecode(srv.response.measurement_image, CV_LOAD_IMAGE_UNCHANGED);

        if (!img_combined.data)
        {
            img_combined = img;
        }
        else
        {
            for(int i = 0; i < img.rows * img.cols; ++i)
            {
                if (img.at<cv::Vec3b>(i) != cv::Vec3b(0, 0, 0))
                {
                    img_combined.at<cv::Vec3b>(i) = img.at<cv::Vec3b>(i);
                }
            }
        }

        cv::Rect roi = getBoundingRect(img);
        rois.push_back(roi);

        cv::rectangle(img_combined, roi, colors[i_color], 2);
        i_color = (i_color + 1) % colors.size();

        ids.push_back(id.str());

        cv::Mat img_cropped = cv::imdecode(srv.response.measurement_image_unmasked, CV_LOAD_IMAGE_UNCHANGED);
        cv::imshow(id.str(), img_cropped);
    }  

    if (!img_combined.data)
    {
        std::cout << "No entities found" << std::endl;
        return 1;
    }

    //Create a window
    cv::namedWindow("Segmentation", 1);

    cv::Mat img_combined_resized;
    cv::resize(img_combined, img_combined_resized, cv::Size(img_combined.cols / 2, img_combined.rows / 2));

    cv::imshow("Segmentation", img_combined_resized);
    cv::waitKey();

    // Try to classify
    ed_perception::Classify srv_classify;
    srv_classify.request.ids = ids;

    if (!cl_perception.call(srv_classify))
    {
        std::cout << "Could not classify" << std::endl;
    }
    else
    {
        for(unsigned int i = 0; i < srv_classify.response.types.size(); ++i)
        {
            std::string type = srv_classify.response.types[i];
            if (type.empty())
                type = "unknown";

            std::cout << rois[i].tl() << ": " << type << std::endl;

            cv::putText(img_combined, type, rois[i].tl(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);
        }
    }

    //set the callback function for any mouse event
    cv::setMouseCallback("Segmentation", mouseCallback, NULL);

    cv::resize(img_combined, img_combined_resized, cv::Size(img_combined.cols / 2, img_combined.rows / 2));
    cv::imshow("Segmentation", img_combined_resized);
    cv::waitKey();

    return 0;
}

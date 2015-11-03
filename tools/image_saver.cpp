#include <ros/ros.h>

// Services
#include <ed/Query.h>
#include <ed_sensor_integration/GetImage.h>

// Write to file
#include <fstream>

// Read RGBD
#include <tue/serialization/input_archive.h>
#include <tue/serialization/conversions.h>
#include <rgbd/serialization.h>

// Time
//#include <stdio.h>
#include <time.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

// Get current date/time, format is YYYY-MM-DD-HH-mm-ss
std::string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Show usage

    std::cout << "Shows RGBD image stream and allows to save images." << std::endl
              << std::endl
              << "Keys:" << std::endl
              << std::endl
              << "       <SPACE>    Save image to disk" << std::endl
              << "             +    Increase frames per second" << std::endl
              << "             -    decrease frames per second" << std::endl
              << "    <ESC> or q    quit" << std::endl
              << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Initialize

    ros::init(argc, argv, "ed_image_saver");
    ros::NodeHandle nh;
    ros::ServiceClient cl_get_image = nh.serviceClient<ed_sensor_integration::GetImage>("ed/kinect/get_image");

    double fps = 1;

    ros::Time t_last_saved(0);

    rgbd::Image image;
    ed_sensor_integration::GetImage srv;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Start loop

    while (ros::ok())
    {
        ros::Time t_now = ros::Time::now();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if it is time to request image

        if (t_now - t_last_saved > ros::Duration(1.0 / fps))
        {
            // - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Retrieve image

            // Try to retrieve the current kinect image
            srv.request.filename = currentDateTime();
            if (!cl_get_image.call(srv))
            {
                std::cout << "Could not call service '" << cl_get_image.getService() << "'." << std::endl;
                ros::Duration(1.0).sleep();
                continue;
            }

            // - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Deserialize rgbd image

            std::stringstream stream;
            tue::serialization::convert(srv.response.rgbd_data, stream);
            tue::serialization::InputArchive a_in(stream);
            rgbd::deserialize(a_in, image);

            t_last_saved = t_now;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Show rgbd image

        cv::Mat img = image.getRGBImage().clone();
        cv::putText(img, "Press <SPACE> to save", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

        std::stringstream s_fps;
        s_fps << fps << " fps";

        cv::putText(img, s_fps.str(), cv::Point(img.cols - 60, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

        cv::imshow("Image Saver", img);
        unsigned char key = cv::waitKey(30);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Deal with key press

        if (key == 27 || key == 'q') // ESC or 'q'
        {
            break;
        }
        else if (key == '+')
        {
            fps = std::min<double>(10, fps + 1);
        }
        else if (key == '-')
        {
            fps = std::max<double>(1, fps - 1);
        }
        else if (key == 32) // SPACE
        {
            const std::string& filename = srv.request.filename;

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Write RGBD file

            std::ofstream f_out;
            std::string rgbd_filename = filename + ".rgbd";
            f_out.open(rgbd_filename.c_str());
            f_out.write(reinterpret_cast<char*>(&srv.response.rgbd_data[0]), srv.response.rgbd_data.size());

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Write JSON file

            std::ofstream f_meta;
            f_meta.open((filename + ".json").c_str());            
            f_meta << srv.response.json_meta_data;

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Show image with filename

            cv::Mat img = image.getRGBImage().clone();
            cv::putText(img, "Saved to " + filename + ".json", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

            cv::imshow("Image Saver", img);
            cv::waitKey(1000);
        }
    }

    return 0;
}

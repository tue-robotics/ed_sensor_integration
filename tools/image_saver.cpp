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
    ros::init(argc, argv, "ed_image_saver");
    ros::NodeHandle nh;
    ros::ServiceClient cl_get_image = nh.serviceClient<ed_sensor_integration::GetImage>("ed/kinect/get_image");
    ros::ServiceClient cl_query_wm = nh.serviceClient<ed::Query>("ed/query");

    std::string wm_filename;

    while (ros::ok())
    {
        // Try to retrieve the current kinect image
        ed_sensor_integration::GetImage srv;
        if (!cl_get_image.call(srv))
        {
            std::cout << "Could not call service '" << cl_get_image.getService() << "'." << std::endl;
            ros::Duration(1.0).sleep();
            continue;
        }

        // ----------------------------------------
        // Deserialize rgbd image

        std::stringstream stream;
        tue::serialization::convert(srv.response.rgbd_data, stream);
        tue::serialization::InputArchive a_in(stream);

        rgbd::Image image;
        rgbd::deserialize(a_in, image);

        // ----------------------------------------
        // Show rgbd image

        cv::imshow("RGB", image.getRGBImage());
        unsigned char key = cv::waitKey(1000);

        if (key == 27 || key == 113) // ESC or 'q'
        {
            break;
        }
        else if (key == 32) // SPACE
        {
            std::string filename = currentDateTime();

            if (wm_filename.empty())
            {
                // If world model has not been stored yet, query it and store it
                ed::Query srv_wm;
                if (cl_query_wm.call(srv_wm))
                {
                    // Save world model
                    wm_filename = filename + ".wm";
                    std::ofstream f_wm;
                    f_wm.open(wm_filename.c_str());
                    f_wm << srv_wm.response.human_readable;
                }
                else
                {
                    std::cout << "Could not call service '" << cl_query_wm.getService() << "'." << std::endl;
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }

            // ----------------------------------------
            // Write RGBD file
            std::ofstream f_out;
            std::string rgbd_filename = filename + ".rgbd";
            f_out.open(rgbd_filename.c_str());
            f_out.write(reinterpret_cast<char*>(&srv.response.rgbd_data[0]), srv.response.rgbd_data.size());

            // ----------------------------------------
            // Write JSON file with extra field containing world_model_filename
            std::string& json_str = srv.response.json_meta_data;
            json_str[json_str.size() - 1] = ','; // Change last '}' into ',' (I know this is ugly...)

            // Open file for writing
            std::ofstream f_meta;
            f_meta.open((filename + ".json").c_str());

            // Write world model filename as extra field
            f_meta << json_str
                   << "\"world_model_filename\":\"" << wm_filename << "\","
                   << "\"rgbd_filename\":\"" << rgbd_filename << "\""
                   << "}";
        }
    }

    return 0;
}

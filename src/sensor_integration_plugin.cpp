#include "sensor_integration_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/world_model/transform_crawler.h>
#include <ed/update_request.h>

#include <ros/subscribe_options.h>
#include <ros/node_handle.h>

#include <rgbd/Image.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

SensorIntegrationPlugin::SensorIntegrationPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

SensorIntegrationPlugin::~SensorIntegrationPlugin()
{
    for(std::vector<rgbd::Client*>::iterator it = rgbd_clients_.begin(); it != rgbd_clients_.end(); ++it)
        delete *it;
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::configure(tue::Configuration config)
{
    ros::NodeHandle nh;

    if (config.read("joints", tue::OPTIONAL))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            if (config.value("topic", topic))
            {
                ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                            topic, 1, boost::bind(&SensorIntegrationPlugin::jointCallback, this, _1), ros::VoidPtr(), &cb_queue_);

                ros::Subscriber joint_sub = nh.subscribe(sub_options);

                joint_subs_.push_back(joint_sub);
            }
        }

        config.endArray();
    }

    if (config.read("kinects", tue::OPTIONAL))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            if (config.value("topic", topic, tue::OPTIONAL))
            {
                // Initialize kinect client
                rgbd::Client* rgbd_client = new rgbd::Client();
                rgbd_client->intialize(topic);
                rgbd_clients_.push_back(rgbd_client);
            }
            else
            {
                std::string rgb_topic, depth_topic, info_topic;
                if (config.value("rgb_topic", rgb_topic) && config.value("depth_topic", depth_topic) && config.value("info_topic", info_topic))
                {
                    // Initialize kinect client
                    rgbd::Client* rgbd_client = new rgbd::Client();
                    rgbd_client->intialize(rgb_topic, depth_topic, info_topic);
                    rgbd_clients_.push_back(rgbd_client);
                }
            }

        }

        config.endArray();
    }

    if (config.read("lasers", tue::OPTIONAL))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            if (config.value("topic", topic))
            {
                std::cout << "[ED Sensor Integration] Cannot deal with laser range finders yet." << std::endl;
            }
        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    update_req_ = &req;
    cb_queue_.callAvailable();

    // Receive new image
    for(std::vector<rgbd::Client*>::iterator it = rgbd_clients_.begin(); it != rgbd_clients_.end(); ++it)
    {
        rgbd::ImagePtr rgbd_image;
        rgbd_image = (*it)->nextImage();
        if (rgbd_image)
        {
            std::cout << "[" << std::fixed << std::setprecision(9) << rgbd_image->getTimestamp() << "] Received image" << std::endl;
            cv::imshow("image", rgbd_image->getRGBImage());
            cv::waitKey(3);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->name.size() != msg->position.size())
    {
        std::cout << "[ED Sensor Integration] On joint callback: name and position vector must be of equal length." << std::endl;
        return;
    }

    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
        const std::string& name = msg->name[i];
        double pos = msg->position[i];

        std::cout << "[" << msg->header.stamp << "] " << name << ": " << pos << std::endl;
    }

}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SensorIntegrationPlugin)

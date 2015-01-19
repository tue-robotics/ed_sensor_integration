#include "sensor_integration_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/world_model/transform_crawler.h>
#include <ed/update_request.h>

#include <ros/subscribe_options.h>
#include <ros/node_handle.h>

#include <rgbd/Image.h>

// Robot model parsing
#include <kdl_parser/kdl_parser.hpp>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

JointInfo::JointInfo()
{
    position_cache.setMaxSize(100); // TODO: get rid of magic number
}

// ----------------------------------------------------------------------------------------------------

bool JointInfo::calculatePosition(const ed::Time& time, float& pos) const
{
    ed::TimeCache<float>::const_iterator it_low, it_high;
    position_cache.getLowerUpper(time, it_low, it_high);

    if (it_low == position_cache.end())
    {
        std::cout << "Joint buffer: too long ago" << std::endl;
        return false;
    }

    if (it_high == position_cache.end())
    {
        std::cout << "Joint buffer: too recent" << std::endl;
        return false;
    }

    // Interpolate
    float p1 = it_low->second;
    float p2 = it_high->second;

    float dt1 = time.seconds() - it_low->first.seconds();
    float dt2 = it_high->first.seconds() - time.seconds();

    // Linearly interpolate joint positions
    pos = (p1 * dt2 + p2 * dt1) / (dt1 + dt2);

    return true;
}

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::string urdf_xml;
    if (config.value("urdf_file", urdf_xml, tue::OPTIONAL))
    {

        if (!kdl_parser::treeFromFile(urdf_xml, tree_))
        {
            config.addError("Could not initialize KDL tree object.");
            return;
        }

//        if (!robot_model_.initString(urdf_xml))
//        {
//            std::cout << "Could not load robot model." << std::endl;
//            return;
//        }

        constructRobot(tree_.getRootSegment());
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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

//            float joint_pos;
//            if (joints_["torso_joint"].calculatePosition(rgbd_image->getTimestamp(), joint_pos))
//                std::cout << "[" << std::fixed << std::setprecision(9) << rgbd_image->getTimestamp() << "] " << joint_pos << std::endl;

            cv::imshow("image", rgbd_image->getRGBImage());
            cv::waitKey(3);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::constructRobot(const KDL::SegmentMap::const_iterator& it_segment)
{
    const KDL::Segment& segment = it_segment->second.segment;

    JointInfo& info = joints_[segment.getJoint().getName()];
    info.segment = segment;

    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for(std::vector<KDL::SegmentMap::const_iterator>::const_iterator it_child = children.begin(); it_child != children.end(); ++it_child)
        constructRobot(*it_child);
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

        std::map<std::string, JointInfo>::iterator it_joint = joints_.find(name);
        if (it_joint != joints_.end())
        {
            JointInfo& info = it_joint->second;
            info.position_cache.insert(msg->header.stamp.toSec(), pos);
            std::cout << "[" << msg->header.stamp << "] " << name << ": " << pos << std::endl;
        }
        else
        {
            std::cout << "[ED Sensor Integration] On joint callback: unknown joint: '" << name << "'." << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SensorIntegrationPlugin)

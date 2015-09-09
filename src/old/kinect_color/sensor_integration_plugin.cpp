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
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::configure(tue::Configuration config)
{
    ros::NodeHandle nh;

    if (!config.value("robot_name", robot_name_))
        return;

    if (config.readArray("joints", tue::OPTIONAL))
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

    if (config.readArray("kinects", tue::OPTIONAL))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            if (config.value("topic", topic, tue::OPTIONAL))
            {
                kinects_.push_back(KinectInfo());
                KinectInfo& kinect = kinects_.back();

                // Initialize kinect client
                kinect.client = new rgbd::Client();
                kinect.client->intialize(topic);
            }
            else
            {
                std::string rgb_topic, depth_topic, info_topic;
                if (config.value("rgb_topic", rgb_topic) && config.value("depth_topic", depth_topic) && config.value("info_topic", info_topic))
                {
                    kinects_.push_back(KinectInfo());
                    KinectInfo& kinect = kinects_.back();

                    // Initialize kinect client
                    kinect.client = new rgbd::Client();
                    kinect.client->intialize(rgb_topic, depth_topic, info_topic);
                }
            }

        }

        config.endArray();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (config.readArray("lasers", tue::OPTIONAL))
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

    color_table_.readFromFile("/home/sdries/ros/hydro/dev/src/ed_sensor_integration/data/color_names.txt");
}

// ----------------------------------------------------------------------------------------------------

void SensorIntegrationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    update_req_ = &req;
    cb_queue_.callAvailable();

    // Receive new image
    for(std::vector<KinectInfo>::iterator it = kinects_.begin(); it != kinects_.end(); ++it)
    {
        KinectInfo& kinect = *it;

        rgbd::ImagePtr rgbd_image;
        rgbd_image = kinect.client->nextImage();
        if (rgbd_image)
        {
            if (!kinect.kinematic_chain)
            {
                if (rgbd_image->getFrameId().empty())
                    continue;

                std::string rel_frame_id;
                if (rgbd_image->getFrameId()[0] != '/')
                    rel_frame_id = rgbd_image->getFrameId();
                else
                    rel_frame_id = rgbd_image->getFrameId().substr(robot_name_.size() + 2);

                kinect.kinematic_chain = new KDL::Chain;
                if (tree_.getChain("base_link", rel_frame_id, *kinect.kinematic_chain))
                {
                    std::cout << "[ED Sensor Integration] Chain found from 'base_link' to '" << rel_frame_id << "'." << std::endl;
                }
                else
                {
                    std::cout << "[ED Sensor Integration] Could not initialize kinematic chain from 'base_link' to '" << rel_frame_id << "'." << std::endl;
                    delete kinect.kinematic_chain;
                    kinect.kinematic_chain = 0;
                }
            }

            std::cout << "[" << std::fixed << std::setprecision(9) << rgbd_image->getTimestamp() << "] Received image" << std::endl;

            cv::Mat rgb = rgbd_image->getRGBImage();

            float neck_pan, neck_tilt;
            if (joints_["neck_pan_joint"].calculatePosition(rgbd_image->getTimestamp(), neck_pan)
                    && joints_["neck_tilt_joint"].calculatePosition(rgbd_image->getTimestamp(), neck_tilt))
            {
                cv::circle(rgb, cv::Point(320, 240), 5, cv::Scalar(0, 0, 255), 2);
                cv::circle(rgb, cv::Point(320, 240) + cv::Point(neck_pan * 100, neck_tilt * 100), 10, cv::Scalar(255, 0, 0), 2);
            }

            cv::Mat depth = rgbd_image->getDepthImage();

            cv::Mat filtered = rgb.clone();

            int w = 3;
            int n = (w + w + 1) * (w + w + 1);

            for(int y = w; y < rgb.rows - w; ++y)
            {
                for(int x = w; x < rgb.cols - w; ++x)
                {
                    const cv::Vec3b& p = rgb.at<cv::Vec3b>(y, x);
                    float p_red = color_table_.rgbToDistribution(p[2], p[1], p[0])[ColorTable::RED];

                    int s = 0;
                    for(int y2 = y - w; y2 <= y + w; ++y2)
                    {
                        for(int x2 = x - w; x2 <= x + w; ++x2)
                        {
                            const cv::Vec3b& p2 = rgb.at<cv::Vec3b>(y2, x2);

                            float p2_red = color_table_.rgbToDistribution(p2[2], p2[1], p2[0])[ColorTable::RED];

                            if ((p_red - p2_red) > 0.2)
                                ++s;
                        }
                    }

                    if (s <= 0.25 * n)
                        filtered.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);

//                    if (color_table_.rgbToDistribution(p[2], p[1], p[0])[ColorTable::GREEN] < 0.3)
//                        filtered.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                }
            }

            cv::imshow("image", rgb);
            cv::imshow("filtered", filtered);
            cv::waitKey(3);

            last_rgbd_image_ = rgbd_image;
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
//            std::cout << "[" << msg->header.stamp << "] " << name << ": " << pos << std::endl;
        }
        else
        {
            std::cout << "[ED Sensor Integration] On joint callback: unknown joint: '" << name << "'." << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SensorIntegrationPlugin)

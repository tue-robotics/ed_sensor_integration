#include "kinect_plugin.h"

#include <rgbd/Image.h>

#include <ros/node_handle.h>

#include <ed/uuid.h>
#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <tue/config/reader.h>

#include <rgbd/View.h>

#include "ed/kinect/association.h"

// GetImage
#include <rgbd/serialization.h>
#include <tue/serialization/conversions.h>
#include <ed/io/json_writer.h>
#include <ed/serialization/serialization.h>

//#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

KinectPlugin::KinectPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

KinectPlugin::~KinectPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;    

    std::string topic;
    if (config.value("topic", topic))
    {
        std::cout << "[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic << "'." << std::endl;
        image_buffer_.initialize(topic);
    }

    // - - - - - - - - - - - - - - - - - -
    // Services

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_get_image_ = nh.advertiseService("kinect/get_image", &KinectPlugin::srvGetImage, this);
    srv_update_ = nh.advertiseService("kinect/update", &KinectPlugin::srvUpdate, this);
}

// ----------------------------------------------------------------------------------------------------

void KinectPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    const ed::WorldModel& world = data.world;   

    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and pose

    last_image_.reset();
    if (!image_buffer_.nextImage("map", last_image_, last_sensor_pose_))
        return;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();

    // - - - - - - - - - - - - - - - - - -

//    cv::imshow("kinect", last_image_->getRGBImage());
//    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvGetImage(ed_sensor_integration::GetImage::Request& req, ed_sensor_integration::GetImage::Response& res)
{
    if (!last_image_)
        return true;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Serialize RGBD image

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    rgbd::serialize(*last_image_, a, rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);
    tue::serialization::convert(stream, res.rgbd_data);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Write meta data

    std::stringstream meta_data;
    ed::io::JSONWriter w(meta_data);

    w.writeGroup("sensor_pose");
    ed::serialize(last_sensor_pose_, w);
    w.endGroup();

    w.finish();

    res.json_meta_data = meta_data.str();

    ROS_INFO_STREAM("[ED KINECT] Requested image. Image size: " << res.rgbd_data.size()
                    << " bytes. Meta-data size: " << res.json_meta_data.size() << " bytes.");

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool KinectPlugin::srvUpdate(ed_sensor_integration::Update::Request& req, ed_sensor_integration::Update::Response& res)
{    
    UpdateResult update_result(*update_req_);
    if (!updater_.update(*world_, *last_image_, last_sensor_pose_, req.update_space_description, update_result))
    {
        res.error_msg = update_result.error.str();
        return true;
    }

    for(unsigned int i = 0; i < update_result.entity_updates.size(); ++i)
    {
        EntityUpdate& e_update = update_result.entity_updates[i];
        if (e_update.is_new)
            res.new_ids.push_back(e_update.id.str());
        else
            res.updated_ids.push_back(e_update.id.str());

        // Lock it, such that is won't be cleared by the clearer plugin
        update_req_->setFlag(e_update.id, "locked");
    }

    for(unsigned int i = 0; i < update_result.removed_entity_ids.size(); ++i)
    {
        res.deleted_ids.push_back(update_result.removed_entity_ids[i].str());
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

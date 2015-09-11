#include "kinect_plugin.h"

#include <rgbd/Image.h>

#include <ros/node_handle.h>

#include <ed/uuid.h>
#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <tue/config/reader.h>

// GetImage
#include <rgbd/serialization.h>
#include <tue/serialization/conversions.h>
#include <ed/io/json_writer.h>
#include <ed/serialization/serialization.h>

#include <opencv2/highgui/highgui.hpp>
#include <geolib/Box.h>

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

    cv::imshow("kinect", last_image_->getRGBImage());
    cv::waitKey(3);
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
    // -------------------------------------
    // Parse space description (split on space)

    const std::string& descr = req.update_space_description;
    std::size_t i_space = descr.find(' ');

    ed::UUID entity_id;
    std::string area_name;

    if (i_space == std::string::npos)
    {
        entity_id = descr;
    }
    else
    {
        area_name = descr.substr(0, i_space);
        entity_id = descr.substr(i_space + 1);
    }

    // -------------------------------------
    // Check for entity and last image existence

    ed::EntityConstPtr e = world_->getEntity(entity_id);

    if (!e)
    {
        res.error_msg = "No such entity: '" + entity_id.str() + "'.";
        return true;
    }
    else if (!e->has_pose())
    {
        res.error_msg = "Entity: '" + entity_id.str() + "' has no pose.";
        return true;
    }
    else if (!last_image_)
    {
        res.error_msg = "No image available.";
        return true;
    }

    // -------------------------------------
    // Update entity position

    FitterData fitter_data;
    fitter_.processSensorData(*last_image_, last_sensor_pose_, fitter_data);

    geo::Pose3D new_pose;
    if (fitter_.estimateEntityPose(fitter_data, *world_, entity_id, e->pose(), new_pose))
    {
        update_req_->setPose(entity_id, new_pose);
    }
    else
    {
        res.error_msg = "Could not determine pose of '" + entity_id.str() + "'.";
        return true;
    }

    // -------------------------------------
    // Segment objects

    if (!area_name.empty())
    {
        // Determine segmentation area (the geometrical shape in which the segmentation should take place)

        geo::Shape shape;
        bool found = false;
        tue::config::Reader r(e->data());

        if (r.readArray("areas"))
        {
            while(r.nextArrayItem())
            {
                std::string a_name;
                if (!r.value("name", a_name) || a_name != area_name)
                    continue;

                if (ed::deserialize(r, "shape", shape))
                {
                    found = true;
                    break;
                }
            }

            r.endArray();
        }

        if (!found)
        {
            res.error_msg = "No area '" + area_name + "' for entity '" + entity_id.str() + "'.";
            return true;
        }
        else if (shape.getMesh().getTriangleIs().empty())
        {
            res.error_msg = "Could not load shape of area '" + area_name + "' for entity '" + entity_id.str() + "'.";
            return true;
        }
        else
        {
            geo::Pose3D shape_pose = last_sensor_pose_.inverse() * new_pose;
            cv::Mat filtered_depth_image;
            segmenter_.calculatePointsWithin(*last_image_, shape, shape_pose, filtered_depth_image);
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(KinectPlugin)

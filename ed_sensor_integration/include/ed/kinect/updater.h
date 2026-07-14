#ifndef ED_KINECT_UPDATER_H_
#define ED_KINECT_UPDATER_H_


#include "ed/kinect/fitter.h"
#include "ed/kinect/segmenter.h"
#include "ed/kinect/entity_update.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tue/config/configuration.h>

#include <filesystem>
#include <map>
#include <string>
#include <vector>
// ----------------------------------------------------------------------------------------------------

struct UpdateRequest
{
    UpdateRequest() : background_padding(0), max_yaw_change(M_PI) {}

    // Symbolic description of area to be updated (e.g. "on_top_of cabinet")
    std::string area_description;

    // When applying background removal, amount of padding given to the world model (the more padding
    // the points are 'cut away')
    double background_padding;

    // When refitting an entity, this states the maximum change in yaw (in radians), i.e., the fitted
    // yaw will deviate at most 'max_yaw_change' from the estimated yaw
    double max_yaw_change;

    // Should the supporting entity be fitted
    bool fit_supporting_entity;
};

// ----------------------------------------------------------------------------------------------------

struct UpdateResult
{
    UpdateResult(ed::UpdateRequest& update_req_) : update_req(update_req_) {}

    std::vector<EntityUpdate> entity_updates;
    std::vector<ed::UUID> removed_entity_ids;
    ed::UpdateRequest& update_req;
    std::stringstream error;
};

// ----------------------------------------------------------------------------------------------------

class Updater
{

public:

    Updater(tue::Configuration config);

    ~Updater();

    bool update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                const UpdateRequest& req, UpdateResult& res);

private:
    /**
     * @brief Publish segmentation results and pointcloud estimation as ROS messages.
     *
     * @param filtered_depth_image The filtered depth image to publish.
     * @param rgb The RGB image to publish.
     * @param sensor_pose The pose of the sensor.
     * @param clustered_images The clustered segmentation masks.
     * @param boxes The bounding boxes to visualize.
     * @param mask_pub_ The ROS publisher for the mask images.
     * @param cloud_pub_ The ROS publisher for the point cloud data.
     * @param res_updates The entity updates to publish.
     */
    void publishSegmentationResults(const cv::Mat& filtered_depth_image, const cv::Mat& rgb,
                                    const geo::Pose3D& sensor_pose, std::vector<cv::Mat>& clustered_images,
                                    const std::vector<cv::Rect>& boxes, std::vector<EntityUpdate>& res_updates);

    Fitter fitter_;

    std::unique_ptr<Segmenter> segmenter_;

    // Stores for each segmented entity with which area description it was found
    std::map<ed::UUID, std::string> id_to_area_description_;

    //For displaying SAM MASK
    ros::Publisher mask_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher box_pub_;
    bool verbose;

};

#endif

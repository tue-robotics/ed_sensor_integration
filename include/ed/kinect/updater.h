#ifndef ED_KINECT_UPDATER_H_
#define ED_KINECT_UPDATER_H_

#include "ed/kinect/fitter.h"
#include "ed/kinect/segmenter.h"
#include "ed/kinect/entity_update.h"

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

    Updater();

    ~Updater();

    bool update(const ed::WorldModel& world, const rgbd::ImageConstPtr& image, const geo::Pose3D& sensor_pose,
                const UpdateRequest& req, UpdateResult& res);

private:

    Fitter fitter_;

    Segmenter segmenter_;

    // Stores for each segmented entity with which area description it was found
    std::map<ed::UUID, std::string> id_to_area_description_;

};

#endif

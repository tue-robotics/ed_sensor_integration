#ifndef ED_KINECT_UPDATER_H_
#define ED_KINECT_UPDATER_H_

#include "ed/kinect/fitter.h"
#include "ed/kinect/segmenter.h"
#include "ed/kinect/entity_update.h"

// ----------------------------------------------------------------------------------------------------

struct UpdateRequest
{
    UpdateRequest() : max_association_distance(0) {}

    std::string area_description;
    double max_association_distance;
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

#ifndef ED_KINECT_UPDATER_H_
#define ED_KINECT_UPDATER_H_

#include "ed/kinect/fitter.h"
#include "ed/kinect/segmenter.h"
#include "ed/kinect/entity_update.h"

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

    bool update(const ed::WorldModel& world, const rgbd::Image& image, const geo::Pose3D& sensor_pose,
                const std::string& update_command, UpdateResult& res);

private:

    Fitter fitter_;

    Segmenter segmenter_;

};

#endif

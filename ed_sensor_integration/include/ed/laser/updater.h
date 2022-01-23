#ifndef ED_LASER_LASER_UPDATER_H_
#define ED_LASER_LASER_UPDATER_H_

#include "ed/laser/entity_update.h"

#include <ed/convex_hull.h>
#include <ed/init_data.h>
#include <ed/world_model.h>

#include <geolib/sensors/LaserRangeFinder.h>

#include <map>
#include <string>

typedef std::vector<unsigned int> ScanSegment;

class LaserUpdater
{

public:
    /**
     *  @brief Configure updater
     *
     *  @param[in] config Configuration which may contain the following fields:
     *      world_association_distance
     *      min_segment_size_pixels
     *      segment_depth_threshold
     *      min_cluster_size
     *      max_cluster_size
     *      max_gap_size
     *      fit_entities Whether or not to fit an entity (0 or 1)
     */
    void configure(tue::Configuration& config);

    /**
     * @brief update update the worldmodel based on a novel laserscan message.
     *
     * @param[in] world worldmodel to be updated
     * @param[in] scan laserscan message
     * @param[in] sensor_pose pose of the sensor at the time of the measurement
     * @param[out] req update request
     */
    void update(const ed::WorldModel& world, std::vector<double>& sensor_ranges,
                const geo::Pose3D& sensor_pose, const double timestamp, ed::UpdateRequest& req);

    /**
     * @brief configure the LRF model based on a laserscan message
     *
     * @param num_beams number of beams in the lrf model
     * @param angle_min angle corresponding to the first beam in radians
     * @param angle_max angle corresponding to the final beam in radians
     * @param range_min minimum distance that can be detected with the lrf in meters
     * @param range_max maximum distance that can be detected with the lrf in meters
     */
    void configureLaserModel(uint num_beams, double angle_min, double angle_max, double range_min, double range_max)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(angle_min, angle_max);
        lrf_model_.setRangeLimits(range_min, range_max);
    }

     /**
     * @brief set the frame of the laser model
     *
     * @param frame_id laserscan message
     */
    void setLaserFrame(std::string frame_id)
    {
        lrf_frame_ = frame_id;
    }

    /**
     * @brief get the number of beams of the model
     */
    unsigned int getNumBeams() { return lrf_model_.getNumBeams(); }

    /**
     * @brief render the worldmodel as would be seen by the lrf.
     *
     * @param[in] sensor_pose pose of the lrf to be modeled in the world frame.
     * @param[in] world worldmodel
     * @param[out] model_ranges ranges of distances as would be seen by an lrf
     */
    void renderWorld(const geo::Pose3D& sensor_pose, const ed::WorldModel& world, std::vector<double>& model_ranges);

    // parameters
    int min_segment_size_pixels_; // in nr of laser points
    double world_association_distance_; // in m
    double segment_depth_threshold_; 
    double min_cluster_size_;
    double max_cluster_size_;
    bool fit_entities_;
    int max_gap_size_;

private:
    /**
     * @brief associate: filter sensor information and remove ranges that can be associated with the worldmodel. Leaving only novel data.
     *
     * @param[in] sensor_ranges distances measured by the lrf
     * @param[in] model_ranges distances as predicted by the worldmodel
     * @param[out] sensor_ranges filtered distances. (associated ranges have value 0.0)
     */
    void associate(std::vector<double>& sensor_ranges, const std::vector<double>& model_ranges);

    /**
     *  @brief divide the sensor ranges into segments
     */
    std::vector<ScanSegment> segment(const std::vector<double>& sensor_ranges);

    /**
     * @brief convert a segment of ranges to a convex hull
     */
    EntityUpdate segmentToConvexHull(const ScanSegment& segment, const geo::Pose3D sensor_pose, const std::vector<double>& sensor_ranges);

    // PARAMETERS
    geo::LaserRangeFinder lrf_model_;
    std::string lrf_frame_;

    std::map<ed::UUID,geo::Pose3D> pose_cache;

};

#endif

#ifndef ed_sensor_integration_kinect_point_normal_alm_h_
#define ed_sensor_integration_kinect_point_normal_alm_h_

#include "ed_sensor_integration/kinect/almodules/rgbd_al_module.h"

#include <ros/publisher.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace edKinect
{

// Define a new point representation for the association
class AssociationPR : public pcl::PointRepresentation <pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

    public:
        AssociationPR ()
        {
            // Define the number of dimensions
            nr_dimensions_ = 6;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
        {
            // < x, y, z, curvature >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.normal_x;
            out[4] = p.normal_y;
            out[5] = p.normal_z;
//            out[3] = p.curvature;
        }
};

class PointNormalALM : public edKinect::RGBDALModule
{

public:

    PointNormalALM();

    void process(const ed::RGBDData& rgbd_data,
                 ed::PointCloudMaskPtr& not_associated_mask,
                 const ed::WorldModel &world_model,
                 ed::UpdateRequest& req);

    void configure(tue::Configuration config);

protected:

    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_;
    AssociationPR point_representation_, point_representation2_;

    //! tunable params
    float position_weight_;
    float normal_weight_;
    float association_correspondence_distance_;
    int render_width_;
    float render_max_range_;
    float render_voxel_size_;
    int normal_k_search_;

};

}

#endif

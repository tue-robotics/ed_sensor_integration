#include "ed_sensor_integration/kinect/almodules/relative_alm.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>
#include <opencv/highgui.h>
#include <pcl/registration/icp.h>

#include <ed/helpers/depth_data_processing.h>
#include <ed/helpers/visualization.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include "ed_sensor_integration/kinect/almodules/world_model_renderer.h"

#include <tue/profiling/scoped_timer.h>

#include <geolib/ros/tf_conversions.h>

namespace edKinect
{

RelativeLocalizationModule::RelativeLocalizationModule() : RGBDALModule("relative_localization")
{
}

void RelativeLocalizationModule::configure(tue::Configuration config)
{
    if (config.readGroup("parameters"))
    {
        config.value("association_correspondence_distance", association_correspondence_distance_);
        config.value("position_weight", position_weight_);
        config.value("normal_weight", normal_weight_);
        config.value("visualize", visualize_);
        config.value("render_width", render_width_);
        config.value("render_max_range", render_max_range_);
        config.value("render_voxel_size", render_voxel_size_);
        config.value("normal_k_search", normal_k_search_);

        std::cout << "Parameters relative localization association module: \n" <<
                     "- association_correspondence_distance: " << association_correspondence_distance_ << "\n" <<
                     "- position_weight: " << position_weight_ << "\n" <<
                     "- normal_weight: " << normal_weight_ << "\n" <<
                     "- render_width: " << render_width_ << "\n" <<
                     "- render_max_range: " << render_max_range_ << "\n" <<
                     "- render_voxel_size_: " << render_voxel_size_ << "\n" <<
                     "- normal_k_search: " << normal_k_search_ << "\n" <<
                     "- visualize: " << visualize_ << std::endl;

        config.endGroup();
    }

    float pw = position_weight_;
    float cw = normal_weight_;
    float alpha[6] = {pw,pw,pw,cw,cw,cw};
    point_representation_.setRescaleValues (alpha);

    tree_ = pcl::KdTreeFLANN<pcl::PointNormal>::Ptr(new pcl::KdTreeFLANN<pcl::PointNormal>);
}

geo::Transform RelativeLocalizationModule::eigenMat2geoTransform(Eigen::Matrix<float,4,4> T) {
    geo::Transform Transformation;

    Transformation.R.xx = T(0,0);
    Transformation.R.xy = T(0,1);
    Transformation.R.xz = T(0,2);

    Transformation.R.yx = T(1,0);
    Transformation.R.yy = T(1,1);
    Transformation.R.yz = T(1,2);

    Transformation.R.zx = T(2,0);
    Transformation.R.zy = T(2,1);
    Transformation.R.zz = T(2,2);

    Transformation.t.x = T(0,3);
    Transformation.t.y = T(1,3);
    Transformation.t.z = T(2,3);
    return Transformation;
}

void RelativeLocalizationModule::process(ed::RGBDData& sensor_data,
                                         ed::PointCloudMaskPtr& not_associated_mask,
                                         const ed::WorldModel& world_model,
                                         ed::UpdateRequest &req)
{

    //! 1) Get the world model point cloud
    pcl::PointCloud<pcl::PointNormal>::ConstPtr world_model_npcl;
    std::vector<const ed::Entity*> world_model_pc_entity_ptrs;
    rgbd::View sensor_view(*sensor_data.image, render_width_);

    // Render the view
    WorldModelRenderer wmr;
    cv::Mat wm_depth_img = cv::Mat::zeros(sensor_view.getHeight(), sensor_view.getWidth(), CV_32F);
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_model_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    wmr.render(sensor_data.sensor_pose, world_model, render_max_range_, sensor_view, wm_depth_img, *world_model_pcl, world_model_pc_entity_ptrs);

    // Downsample the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr world_model_pcl_downsampled = ed::helpers::ddp::downSamplePcl(world_model_pcl, render_voxel_size_);

    // Calculate the normals with use of pcl
    world_model_npcl =  ed::helpers::ddp::pclToNpcl(world_model_pcl_downsampled, normal_k_search_);

    if (visualize_) {
        ed::helpers::visualization::publishPclVisualizationMarker(sensor_data.sensor_pose, world_model_pcl_downsampled, vis_marker_pub_, 1, "world_model_pc");
    }

    //! 3.1) Match all depth data with world model render to update sensor pose
    pcl::PointCloud<pcl::PointNormal>::Ptr final_pc(new pcl::PointCloud<pcl::PointNormal>);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(sensor_data.point_cloud_with_normals);
    icp.setInputTarget(world_model_npcl);
    icp.align(*final_pc);

    if (icp.hasConverged()) {
        Eigen::Matrix<float, 4, 4> T = icp.getFinalTransformation();
        geo::Pose3D pose_correction = RelativeLocalizationModule::eigenMat2geoTransform(T);
        geo::Pose3D pose_corrected;

        pose_corrected = sensor_data.sensor_pose * pose_correction;

        if (visualize_) {
            ed::helpers::visualization::publishRGBDViewFrustrumVisualizationMarker(sensor_view, pose_corrected, vis_marker_pub_, 2, "corrected_pose");
            ed::helpers::visualization::publishPclVisualizationMarker(pose_corrected, final_pc, vis_marker_pub_, 2, "corrected_pc");
        }

        // Convert back from Geolib frame to ROS frame
        pose_corrected.R = pose_corrected.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

        // Set pose of improved kinect pose entity
        req.setPose("improved_kinect_frame", pose_corrected); //TODO: Remove hard coded tf name

    }

    if (visualize_) {
        ed::helpers::visualization::publishRGBDViewFrustrumVisualizationMarker(sensor_view, sensor_data.sensor_pose, vis_marker_pub_, 1, "original_pose");
    }

    pub_profile_.publish();
} // process method

} // namespace ed

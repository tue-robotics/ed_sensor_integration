#include "ed_sensor_integration/kinect/segmodules/euclidean_clustering_sm.h"

#include <ed/helpers/depth_data_processing.h>
#include <ed/helpers/visualization.h>
#include <ed/mask.h>

namespace edKinect
{

EuclideanClusteringSM::EuclideanClusteringSM() : RGBDSegModule("euclidean_clustering")
{

}

void EuclideanClusteringSM::configure(tue::Configuration config)
{
    if (config.readGroup("parameters"))
    {
        config.value("tolerance", tolerance_);
        config.value("min_cluster_size", min_cluster_size_);

        std::cout << "Parameters euclidean clustering: \n" <<
        "- tolerance: " << tolerance_ << "\n" <<
        "- min_cluster_size: " << min_cluster_size_ << std::endl;

        config.endGroup();
    }
}

void EuclideanClusteringSM::process(const ed::RGBDData& rgbd_data, std::vector<ed::PointCloudMaskPtr>& segments)
{
    // Copy and clear segments
    std::vector<ed::PointCloudMaskPtr> old_segments = segments;
    segments.clear();

    // Loop over all segments and perform euclidean cluster segmentation per segment
    for (std::vector<ed::PointCloudMaskPtr>::const_iterator it = old_segments.begin(); it != old_segments.end(); ++it) {

        const ed::PointCloudMaskPtr& old_seg = *it;

        // Get clusters from pcl
        std::vector<ed::PointCloudMaskPtr> clusters;
        ed::helpers::ddp::findEuclideanClusters(rgbd_data.point_cloud, old_seg, tolerance_, min_cluster_size_, clusters);

        // Iter over clusters and add segments
        for (std::vector<ed::PointCloudMaskPtr>::const_iterator cit = clusters.begin(); cit != clusters.end(); ++cit)
            segments.push_back(*cit);
    }
}

}

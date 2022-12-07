/*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2011, Willow Garage, Inc.
  *  Copyright (c) 2012-, Open Perception, Inc.
  *
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the copyright holder(s) nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  * $Id$
  *
  */

#pragma once

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/distances.h>

namespace pcl
{
/** \brief @b SampleConsensusModelCircle defines a model for 2D circle from 3D samples (ignoring the third dimension).
 * The model coefficients are defined as:
 *   - \b point_on_axis.x  : the X coordinate of the circle's center point
 *   - \b point_on_axis.y  : the Y coordinate of the circle's center point
 *   - \b radius           : the circle's radius
 *
 * \author Thijs Beurskens
 * \ingroup sample_consensus
 */
template <typename PointT>
class SampleConsensusModelCircle : public SampleConsensusModel<PointT>
{
public:
    using SampleConsensusModel<PointT>::model_name_;
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::radius_min_;
    using SampleConsensusModel<PointT>::radius_max_;
    using SampleConsensusModel<PointT>::error_sqr_dists_;

    using PointCloud = typename SampleConsensusModel<PointT>::PointCloud;
    using PointCloudPtr = typename SampleConsensusModel<PointT>::PointCloudPtr;
    using PointCloudConstPtr = typename SampleConsensusModel<PointT>::PointCloudConstPtr;

    using Ptr = shared_ptr<SampleConsensusModelCircle<PointT> >;
    using ConstPtr = shared_ptr<const SampleConsensusModelCircle<PointT>>;

    /** \brief Constructor for base SampleConsensusModelCylinder.
     * \param[in] cloud the input point cloud dataset
     * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    SampleConsensusModelCircle (const PointCloudConstPtr &cloud, bool random = false)
        : SampleConsensusModel<PointT> (cloud, random)
    {
        model_name_ = "SampleConsensusModelCircle";
        sample_size_ = 3;
        model_size_ = 3;
    }

    /** \brief Constructor for base SampleConsensusModelCylinder.
     * \param[in] cloud the input point cloud dataset
     * \param[in] indices a vector of point indices to be used from \a cloud
     * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
     */
    SampleConsensusModelCircle (const PointCloudConstPtr &cloud,
                                const Indices &indices,
                                bool random = false)
        : SampleConsensusModel<PointT> (cloud, indices, random)
    {
        model_name_ = "SampleConsensusModelCircle";
        sample_size_ = 3;
        model_size_ = 3;
    }

    /** \brief Copy constructor.
     * \param[in] source the model to copy into this
     */
    SampleConsensusModelCircle (const SampleConsensusModelCircle &source) :
        SampleConsensusModel<PointT> ()
    {
        *this = source;
        model_name_ = "SampleConsensusModelCircle";
    }

    /** \brief Empty destructor */
    ~SampleConsensusModelCircle () {}

    /** \brief Copy constructor.
         * \param[in] source the model to copy into this
         */
    inline SampleConsensusModelCircle&
    operator = (const SampleConsensusModelCircle &source)
    {
        SampleConsensusModel<PointT>::operator=(source);
        return (*this);
    }

    /** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients
     * from these samples and store them in model_coefficients. The cylinder coefficients are: point_on_axis,
     * axis_direction, cylinder_radius_R
     * \param[in] samples the point indices found as possible good candidates for creating a valid model
     * \param[out] model_coefficients the resultant model coefficients
     */
    bool
    computeModelCoefficients (const Indices &samples,
                              Eigen::VectorXf &model_coefficients) const override;

    /** \brief Compute all distances from the cloud data to a given cylinder model.
     * \param[in] model_coefficients the coefficients of a cylinder model that we need to compute distances to
     * \param[out] distances the resultant estimated distances
     */
    void
    getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                         std::vector<double> &distances) const override;

    /** \brief Select all the points which respect the given model coefficients as inliers.
     * \param[in] model_coefficients the coefficients of a cylinder model that we need to compute distances to
     * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
     * \param[out] inliers the resultant model inliers
     */
    void
    selectWithinDistance (const Eigen::VectorXf &model_coefficients,
                          const double threshold,
                          Indices &inliers) override;

    /** \brief Count all the points which respect the given model coefficients as inliers.
     *
     * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
     * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
     * \return the resultant number of inliers
     */
    std::size_t
    countWithinDistance (const Eigen::VectorXf &model_coefficients,
                         const double threshold) const override;

    /** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
     * @note: these are the coefficients of the cylinder model after refinement (e.g. after SVD)
     * \param[in] inliers the data inliers found as supporting the model
     * \param[in] model_coefficients the initial guess for the optimization
     * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
     */
    void
    optimizeModelCoefficients (const Indices &inliers,
                               const Eigen::VectorXf &model_coefficients,
                               Eigen::VectorXf &optimized_coefficients) const override;


    /** \brief Create a new point cloud with inliers projected onto the cylinder model.
     * \param[in] inliers the data inliers that we want to project on the cylinder model
     * \param[in] model_coefficients the coefficients of a cylinder model
     * \param[out] projected_points the resultant projected points
     * \param[in] copy_data_fields set to true if we need to copy the other data fields
     */
    void
    projectPoints (const Indices &inliers,
                   const Eigen::VectorXf &model_coefficients,
                   PointCloud &projected_points,
                   bool copy_data_fields = true) const override;

    /** \brief Verify whether a subset of indices verifies the given cylinder model coefficients.
     * \param[in] indices the data indices that need to be tested against the cylinder model
     * \param[in] model_coefficients the cylinder model coefficients
     * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
     */
    bool
    doSamplesVerifyModel (const std::set<int> &indices,
                          const Eigen::VectorXf &model_coefficients,
                          const double threshold) const override;

    /** \brief Return a unique id for this model (SACMODEL_CYLINDER). */
    inline pcl::SacModel
    getModelType () const override { return (SACMODEL_CYLINDER); } // #TODO cannot add a unique identifier

protected:
    using SampleConsensusModel<PointT>::sample_size_;
    using SampleConsensusModel<PointT>::model_size_;

    /** \brief Get the distance from a point to a line (represented by a point and a direction)
     * \param[in] pt a point
     * \param[in] model_coefficients the line coefficients (a point on the line, line direction)
     */
    double
    pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients) const;

    /** \brief Project a point onto a line given by a point and a direction vector
     * \param[in] pt the input point to project
     * \param[in] line_pt the point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
     * \param[in] line_dir the direction of the line (make sure that line_dir[3] = 0 as there are no internal checks!)
     * \param[out] pt_proj the resultant projected point
     */
    inline void
    projectPointToLine (const Eigen::Vector4f &pt,
                        const Eigen::Vector4f &line_pt,
                        const Eigen::Vector4f &line_dir,
                        Eigen::Vector4f &pt_proj) const
    {
        float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);
        // Calculate the projection of the point on the line
        pt_proj = line_pt + k * line_dir;
    }

    /** \brief Project a point onto a cylinder given by its model coefficients (point_on_axis, axis_direction,
     * cylinder_radius_R)
     * \param[in] pt the input point to project
     * \param[in] model_coefficients the coefficients of the cylinder (point_on_axis, axis_direction, cylinder_radius_R)
     * \param[out] pt_proj the resultant projected point
     */
    void
    projectPointToRectangle (const Eigen::Vector4f &pt,
                             const Eigen::VectorXf &model_coefficients,
                             Eigen::Vector4f &pt_proj) const;

    /** \brief Check whether a model is valid given the user constraints.
     * \param[in] model_coefficients the set of model coefficients
     */
    bool
    isModelValid (const Eigen::VectorXf &model_coefficients) const override;

    /** \brief Check if a sample of indices results in a good sample of points
     * indices. Pure virtual.
     * \param[in] samples the resultant index samples
     */
    bool
    isSampleGood (const Indices &samples) const override;

private:
    /** \brief Functor for the optimization function */
    struct OptimizationFunctor : pcl::Functor<float>
    {
        /** Functor constructor
         * \param[in] indices the indices of data points to evaluate
         * \param[in] estimator pointer to the estimator object
         */
        OptimizationFunctor (const pcl::SampleConsensusModelCircle<PointT> *model, const Indices& indices) :
            pcl::Functor<float> (indices.size ()), model_ (model), indices_ (indices) {}

        /** Cost function to be minimized
         * \param[in] x variables array
         * \param[out] fvec resultant functions evaluations
         * \return 0
         */
        int
        operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
            Eigen::Vector4f line_pt  (x[0], x[1], x[2], 0);
            Eigen::Vector4f line_dir (x[3], x[4], x[5], 0);

            for (int i = 0; i < values (); ++i)
            {
                // dist = f - r
                Eigen::Vector4f pt = (*model_->input_)[indices_[i]].getVector4fMap();
                pt[3] = 0;

                fvec[i] = static_cast<float> (pcl::sqrPointToLineDistance (pt, line_pt, line_dir) - x[6]*x[6]);
            }
            return (0);
        }

        const pcl::SampleConsensusModelCircle<PointT> *model_;
        const Indices &indices_;
    };
};
}

//#ifdef PCL_NO_PRECOMPILE
#include "ed_sensor_integration/sac_model_circle.hpp"
//#endif
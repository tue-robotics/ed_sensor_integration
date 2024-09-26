/*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010, Willow Garage, Inc.
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
  a
  */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_HORIZONTAL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_HORIZONTAL_PLANE_H_

#include "ed_sensor_integration/sac_model_horizontal_plane.h"

//////////////////////////////////////////////////////////////////////////

template <typename PointT> bool
pcl::SampleConsensusModelHorizontalPlane<PointT>::computeModelCoefficients (
        const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
    model_coefficients.resize (model_size_);
    model_coefficients[0] = 0;
    model_coefficients[1] = 0;
    model_coefficients[2] = -1;
    model_coefficients[3] = (*input_)[samples[0]].z;
    //std::cout <<model_coefficients << std::endl;
    return (true);
}

template <typename PointT> void
pcl::SampleConsensusModelHorizontalPlane<PointT>::selectWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
    // Check if the model is valid given the user constraints
    if (!isModelValid (model_coefficients))
    {
        inliers.clear ();
        return;
    }

    SampleConsensusModelPlane<PointT>::selectWithinDistance (model_coefficients, threshold, inliers);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelHorizontalPlane<PointT>::countWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold) const
{
    // Check if the model is valid given the user constraints
    if (!isModelValid (model_coefficients))
    {
        return (0);
    }
    //std::cout << model_coefficients << std::endl;
    //std::cout << SampleConsensusModelPlane<PointT>::countWithinDistance (model_coefficients, threshold) << std::endl;
    return (SampleConsensusModelPlane<PointT>::countWithinDistance (model_coefficients, threshold));
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelHorizontalPlane<PointT>::getDistancesToModel (
        const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
    // Check if the model is valid given the user constraints
    if (!isModelValid (model_coefficients))
    {
        distances.clear ();
        return;
    }

    SampleConsensusModelPlane<PointT>::getDistancesToModel (model_coefficients, distances);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelHorizontalPlane<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
    if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    {
        return (false);
    }
    /*
   // Check against template, if given
   if (eps_angle_ > 0.0)
   {
     // Obtain the plane normal
     Eigen::Vector4f coeff = model_coefficients;
     coeff[3] = 0.0f;
     coeff.normalize ();

     Eigen::Vector4f axis (axis_[0], axis_[1], axis_[2], 0.0f);
     if (std::abs (axis.dot (coeff)) > sin_angle_)
     {
       PCL_DEBUG ("[pcl::SampleConsensusModelHorizontalPlane::isModelValid] Angle between plane normal and given axis is too large.\n");
       return  (false);
     }
   }
  */
    return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelHorizontalPlane(T) template class PCL_EXPORTS pcl::SampleConsensusModelHorizontalPlane<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_HORIZONTAL_PLANE_H_

/*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2009-2010, Willow Garage, Inc.
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include "ed_sensor_integration/sac_model_circle.h"
#include <pcl/common/common.h> // for getAngle3D
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle<PointT>::isSampleGood (const Indices &samples) const
{
    if (samples.size () != sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
        return (false);
    }
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle<PointT>::computeModelCoefficients (
        const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
    // Need 3 samples
    if (samples.size () != sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
        return (false);
    }

    if (std::abs ((*input_)[samples[0]].x - (*input_)[samples[1]].x) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[0]].y - (*input_)[samples[1]].y) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[0]].z - (*input_)[samples[1]].z) <= std::numeric_limits<float>::epsilon () ||
            std::abs ((*input_)[samples[1]].x - (*input_)[samples[2]].x) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[1]].y - (*input_)[samples[2]].y) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[1]].z - (*input_)[samples[2]].z) <= std::numeric_limits<float>::epsilon () ||
            std::abs ((*input_)[samples[0]].x - (*input_)[samples[2]].x) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[0]].y - (*input_)[samples[2]].y) <= std::numeric_limits<float>::epsilon () &&
            std::abs ((*input_)[samples[0]].z - (*input_)[samples[2]].z) <= std::numeric_limits<float>::epsilon ())
    {
        return (false);
    }
    Eigen::Vector2f p1 ((*input_)[samples[0]].x, (*input_)[samples[0]].y);
    Eigen::Vector2f p2 ((*input_)[samples[1]].x, (*input_)[samples[1]].y);
    Eigen::Vector2f p3 ((*input_)[samples[2]].x, (*input_)[samples[2]].y);

    float x, y, l, r;

    float denominator = 2*(p1(0)*(p2(1)-p3(1)) - p1(1)*(p2(0)-p3(0)) + p2(0)*p3(1) - p3(0)*p2(1));
    x = (p1.dot(p1)*(p2(1)-p3(1)) + p2.dot(p2)*(p3(1)-p1(1)) + p3.dot(p3)*(p1(1)-p2(1))) / denominator;
    y = (p1.dot(p1)*(p2(0)-p3(0)) + p2.dot(p2)*(p3(0)-p1(0)) + p3.dot(p3)*(p1(0)-p2(0))) / denominator;

    Eigen::Vector2f d;
    d(0) = x-p1(0);
    d(1) = y-p1(1);
    r = sqrt(d.dot(d));

    //save model coefficients
    model_coefficients.resize (model_size_);
    model_coefficients[0] = x;
    model_coefficients[1] = y;
    model_coefficients[2] = r;

    PCL_DEBUG ("[pcl::SampleConsensusModelCircle::computeModelCoefficients] Model is (%g,%g,%g).\n",
               model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle<PointT>::getDistancesToModel (
        const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
    // Check if the model is valid given the user constraints
    if (!isModelValid (model_coefficients))
    {
        distances.clear ();
        return;
    }

    distances.resize (indices_->size ());

    float x = model_coefficients[0], y = model_coefficients[1], r = model_coefficients[2];

    Eigen::Vector2f p1, c1;
    p1(0) = x;
    p1(1) = y;

    for (std::size_t i = 0; i < indices_->size (); ++i)
    {
        Eigen::Vector2f in ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y);
        c1 = p1 - in;
        distances[i] = std::abs(sqrt(c1.dot(c1))-r);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle<PointT>::selectWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
    std::vector<double> distances;
    getDistancesToModel (model_coefficients, distances);
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] < threshold) {inliers.push_back ((*indices_)[i]);}
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle<PointT>::countWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold) const
{
    std::vector<double> distances;
    getDistancesToModel (model_coefficients, distances);
    int count = 0;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] <= threshold) {count++;}
    }
    return (count);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle<PointT>::optimizeModelCoefficients (
        const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
    optimized_coefficients = model_coefficients;

    // Needs a set of valid model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle::optimizeModelCoefficients] Given model is invalid!\n");
        return;
    }

    // Need more than the minimum sample size to make a difference
    if (inliers.size () <= sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle:optimizeModelCoefficients] Not enough inliers found to optimize model coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
        return;
    }

    OptimizationFunctor functor (this, inliers);
    Eigen::NumericalDiff<OptimizationFunctor > num_diff (functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
    int info = lm.minimize (optimized_coefficients);

    // Compute the L2 norm of the residuals
    PCL_DEBUG ("[pcl::SampleConsensusModelCircle::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g\n",
               info, lm.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
            model_coefficients[4], model_coefficients[5], model_coefficients[6], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2], optimized_coefficients[3], optimized_coefficients[4], optimized_coefficients[5], optimized_coefficients[6]);

    Eigen::Vector3f line_dir (optimized_coefficients[3], optimized_coefficients[4], optimized_coefficients[5]);
    line_dir.normalize ();
    optimized_coefficients[3] = line_dir[0];
    optimized_coefficients[4] = line_dir[1];
    optimized_coefficients[5] = line_dir[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle<PointT>::projectPoints (
        const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
    // Needs a valid set of model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle::projectPoints] Given model is invalid!\n");
        return;
    }

    projected_points.header = input_->header;
    projected_points.is_dense = input_->is_dense;

    Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
    Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
    float ptdotdir = line_pt.dot (line_dir);
    float dirdotdir = 1.0f / line_dir.dot (line_dir);

    // Copy all the data fields from the input cloud to the projected one?
    if (copy_data_fields)
    {
        // Allocate enough space and copy the basics
        projected_points.resize (input_->size ());
        projected_points.width    = input_->width;
        projected_points.height   = input_->height;

        using FieldList = typename pcl::traits::fieldList<PointT>::type;
        // Iterate over each point
        for (std::size_t i = 0; i < projected_points.size (); ++i)
            // Iterate over each dimension
            pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[i], projected_points[i]));

        // Iterate through the 3d points and calculate the distances from them to the cylinder
        for (const auto &inlier : inliers)
        {
            Eigen::Vector4f p ((*input_)[inlier].x,
                               (*input_)[inlier].y,
                               (*input_)[inlier].z,
                               1);

            float k = (p.dot (line_dir) - ptdotdir) * dirdotdir;

            pcl::Vector4fMap pp = projected_points[inlier].getVector4fMap ();
            pp.matrix () = line_pt + k * line_dir;

            Eigen::Vector4f dir = p - pp;
            dir[3] = 0.0f;
            dir.normalize ();

            // Calculate the projection of the point onto the cylinder
            pp += dir * model_coefficients[6];
        }
    }
    else
    {
        // Allocate enough space and copy the basics
        projected_points.resize (inliers.size ());
        projected_points.width    = inliers.size ();
        projected_points.height   = 1;

        using FieldList = typename pcl::traits::fieldList<PointT>::type;
        // Iterate over each point
        for (std::size_t i = 0; i < inliers.size (); ++i)
            // Iterate over each dimension
            pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));

        // Iterate through the 3d points and calculate the distances from them to the cylinder
        for (std::size_t i = 0; i < inliers.size (); ++i)
        {
            pcl::Vector4fMap pp = projected_points[i].getVector4fMap ();
            pcl::Vector4fMapConst p = (*input_)[inliers[i]].getVector4fMap ();

            float k = (p.dot (line_dir) - ptdotdir) * dirdotdir;
            // Calculate the projection of the point on the line
            pp.matrix () = line_pt + k * line_dir;

            Eigen::Vector4f dir = p - pp;
            dir[3] = 0.0f;
            dir.normalize ();

            // Calculate the projection of the point onto the cylinder
            pp += dir * model_coefficients[6];
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle<PointT>::doSamplesVerifyModel (
        const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
    // Needs a valid model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelCircle::doSamplesVerifyModel] Given model is invalid!\n");
        return (false);
    }

    for (const auto &index : indices)
    {
        // Approximate the distance from the point to the cylinder as the difference between
        // dist(point,cylinder_axis) and cylinder radius
        // @note need to revise this.
        Eigen::Vector4f pt ((*input_)[index].x, (*input_)[index].y, (*input_)[index].z, 0.0f);
        if (std::abs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]) > threshold)
            return (false);
    }
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::SampleConsensusModelCircle<PointT>::pointToLineDistance (
        const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients) const
{
    Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
    Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
    return sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle<PointT>::projectPointToRectangle (
        const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients, Eigen::Vector4f &pt_proj) const
{
    Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
    Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);

    float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) * line_dir.dot (line_dir);
    pt_proj = line_pt + k * line_dir;

    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the projection of the point onto the cylinder
    pt_proj += dir * model_coefficients[6];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO do not overwrite we dont change it.
template <typename PointT> bool
pcl::SampleConsensusModelCircle<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
    if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
        return (false);
    return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCircle(PointT) template class PCL_EXPORTS pcl::SampleConsensusModelCircle<PointT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_

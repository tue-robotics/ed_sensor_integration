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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_DOUBLE_LINE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_DOUBLE_LINE_H_

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include "ed_sensor_integration/sac_model_double_line.h"
#include <pcl/common/common.h> // for getAngle3D
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelDoubleLine<PointT>::isSampleGood (const Indices &samples) const
{
    if (samples.size () != sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
        return (false);
    }
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelDoubleLine<PointT>::computeModelCoefficients (
        const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
    // Need 3 samples
    if (samples.size () != sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
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

    float x, y, l, w, r;

    x = p1(0), y = p1(1); //output: x and y values

    Eigen::Vector2f c1, c2;

    c1 = p2 - p1; //vector between point 1 and 2
    l = sqrt(c1.dot(c1)); //define length as distance between point 1 and 2
    // determine rotation:
    r = std::atan2(c1(1), c1(0)); //output: orientation of the double_line

    c2 = p1 - p3; //vector between point 3 and 1
    w = (c1(0)*c2(1)-c2(0)*c1(1))/l; //output: width defined as distance between c1 and point 3, signed

    //save model coefficients
    model_coefficients.resize (model_size_);
    model_coefficients[0] = x;
    model_coefficients[1] = y;
    model_coefficients[2] = w;
    model_coefficients[3] = r;

    //std::cout << w << std::endl;

    PCL_DEBUG ("[pcl::SampleConsensusModelDoubleLine::computeModelCoefficients] Model is (%g,%g,%g,%g,%g).\n",
               model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3]);
    //std::cout << "P1: " << std::endl << p1 << std::endl << "P2: " << std::endl << p2 << std::endl << "P3: " << std::endl << p3 << std::endl << model_coefficients << std::endl;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelDoubleLine<PointT>::getDistancesToModel (
        const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
    // Check if the model is valid given the user constraints
    if (!isModelValid (model_coefficients))
    {
        distances.clear ();
        return;
    }

    distances.resize (indices_->size ());

    float x = model_coefficients[0];
    float y = model_coefficients[1];
    float w = model_coefficients[2];
    float r = model_coefficients[3];
    float d1, d2;

    Eigen::Vector2f p1, p2, p3, p4, c1, c2;

    // #TODO Calculation of distances without needing to define points
    //define edge vectors
    c1(0)=std::cos(r);
    c1(1)=std::sin(r);

    c2(0)= std::sin(r)*w;
    c2(1)=-std::cos(r)*w;

    //define corner vectors
    p1(0)=x;
    p1(1)=y;
    p2 = p1 + c1;
    p3 = p2 + c2;
    p4 = p1 + c2;

    for (std::size_t i = 0; i < indices_->size (); ++i)
    {
        Eigen::Vector2f in ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y);

        //compute distances to lines
        d1 = std::abs(c1(0)*(p1(1)-in(1))-(p1(0)-in(0))*c1(1)); //Distance between line 1 and the point
        d2 = std::abs(c1(0)*(p3(1)-in(1))-(p3(0)-in(0))*c1(1)); //Distance between line 2 and the point

        distances[i] = std::min(d1, d2); //smallest of d1 and d2
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelDoubleLine<PointT>::selectWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
    std::vector<double> distances;
    getDistancesToModel (model_coefficients, distances);
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] < threshold) {inliers.push_back ((*indices_)[i]);}
    }

    //std::cout << model_coefficients[2] << std::endl;
    /*
   // Check if the model is valid given the user constraints
   if (!isModelValid (model_coefficients))
   {
     inliers.clear ();
     return;
   }

   inliers.clear ();
   error_sqr_dists_.clear ();
   inliers.reserve (indices_->size ());
   error_sqr_dists_.reserve (indices_->size ());

   Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
   Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
   float ptdotdir = line_pt.dot (line_dir);
   float dirdotdir = 1.0f / line_dir.dot (line_dir);
   // Iterate through the 3d points and calculate the distances from them to the sphere
   for (std::size_t i = 0; i < indices_->size (); ++i)
   {
     // Approximate the distance from the point to the cylinder as the difference between
     // dist(point,cylinder_axis) and cylinder radius
     Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z, 0.0f);
     const double weighted_euclid_dist = (1.0 - normal_distance_weight_) * std::abs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);
     if (weighted_euclid_dist > threshold) // Early termination: cannot be an inlier
       continue;

     // Calculate the point's projection on the cylinder axis
     float k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
     Eigen::Vector4f pt_proj = line_pt + k * line_dir;
     Eigen::Vector4f dir = pt - pt_proj;
     dir.normalize ();

     // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
     Eigen::Vector4f n  ((*normals_)[(*indices_)[i]].normal[0], (*normals_)[(*indices_)[i]].normal[1], (*normals_)[(*indices_)[i]].normal[2], 0.0f);
     double d_normal = std::abs (getAngle3D (n, dir));
     d_normal = (std::min) (d_normal, M_PI - d_normal);

     double distance = std::abs (normal_distance_weight_ * d_normal + weighted_euclid_dist);
     if (distance < threshold)
     {
       // Returns the indices of the points whose distances are smaller than the threshold
       inliers.push_back ((*indices_)[i]);
       error_sqr_dists_.push_back (distance);
     }
   }*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelDoubleLine<PointT>::countWithinDistance (
        const Eigen::VectorXf &model_coefficients, const double threshold) const
{
    std::vector<double> distances;
    getDistancesToModel (model_coefficients, distances);
    int count = 0;
    for (int i = 0; i < distances.size(); i++)
    {
        if (distances[i] <= threshold) {count++;}
    }
    //std::cout << count << std::endl;
    return (count);
    /*// Check if the model is valid given the user constraints
   if (!isModelValid (model_coefficients))
     return (0);

   std::size_t nr_p = 0;

   Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
   Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
   float ptdotdir = line_pt.dot (line_dir);
   float dirdotdir = 1.0f / line_dir.dot (line_dir);
   // Iterate through the 3d points and calculate the distances from them to the sphere
   for (std::size_t i = 0; i < indices_->size (); ++i)
   {
     // Approximate the distance from the point to the cylinder as the difference between
     // dist(point,cylinder_axis) and cylinder radius
     Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z, 0.0f);
     const double weighted_euclid_dist = (1.0 - normal_distance_weight_) * std::abs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);
     if (weighted_euclid_dist > threshold) // Early termination: cannot be an inlier
       continue;

     // Calculate the point's projection on the cylinder axis
     float k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
     Eigen::Vector4f pt_proj = line_pt + k * line_dir;
     Eigen::Vector4f dir = pt - pt_proj;
     dir.normalize ();

     // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
     Eigen::Vector4f n  ((*normals_)[(*indices_)[i]].normal[0], (*normals_)[(*indices_)[i]].normal[1], (*normals_)[(*indices_)[i]].normal[2], 0.0f);
     double d_normal = std::abs (getAngle3D (n, dir));
     d_normal = (std::min) (d_normal, M_PI - d_normal);

     if (std::abs (normal_distance_weight_ * d_normal + weighted_euclid_dist) < threshold)
       nr_p++;
   }
   return (nr_p);*/
    return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelDoubleLine<PointT>::optimizeModelCoefficients (
        const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
    optimized_coefficients = model_coefficients;

    // Needs a set of valid model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine::optimizeModelCoefficients] Given model is invalid!\n");
        return;
    }

    // Need more than the minimum sample size to make a difference
    if (inliers.size () <= sample_size_)
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine:optimizeModelCoefficients] Not enough inliers found to optimize model coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
        return;
    }

    OptimizationFunctor functor (this, inliers);
    Eigen::NumericalDiff<OptimizationFunctor > num_diff (functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
    int info = lm.minimize (optimized_coefficients);

    // Compute the L2 norm of the residuals
    PCL_DEBUG ("[pcl::SampleConsensusModelDoubleLine::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g\n",
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
pcl::SampleConsensusModelDoubleLine<PointT>::projectPoints (
        const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
    // Needs a valid set of model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine::projectPoints] Given model is invalid!\n");
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
pcl::SampleConsensusModelDoubleLine<PointT>::doSamplesVerifyModel (
        const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
    // Needs a valid model coefficients
    if (!isModelValid (model_coefficients))
    {
        PCL_ERROR ("[pcl::SampleConsensusModelDoubleLine::doSamplesVerifyModel] Given model is invalid!\n");
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
pcl::SampleConsensusModelDoubleLine<PointT>::pointToLineDistance (
        const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients) const
{
    Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
    Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
    return sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelDoubleLine<PointT>::projectPointToRectangle (
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
template <typename PointT> bool
pcl::SampleConsensusModelDoubleLine<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
    if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
        return (false);
    /*
   // Check against template, if given
   if (eps_angle_ > 0.0)
   {
     // Obtain the cylinder direction
     const Eigen::Vector3f coeff(model_coefficients[3], model_coefficients[4], model_coefficients[5]);

     double angle_diff = std::abs (getAngle3D (axis_, coeff));
     angle_diff = (std::min) (angle_diff, M_PI - angle_diff);
     // Check whether the current cylinder model satisfies our angle threshold criterion with respect to the given axis
     if (angle_diff > eps_angle_)
     {
       PCL_DEBUG ("[pcl::SampleConsensusModelDoubleLine::isModelValid] Angle between cylinder direction and given axis is too large.\n");
       return (false);
     }
   }

   if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[6] < radius_min_)
   {
     PCL_DEBUG ("[pcl::SampleConsensusModelDoubleLine::isModelValid] Radius is too small: should be larger than %g, but is %g.\n",
                radius_min_, model_coefficients[6]);
     return (false);
   }
   if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[6] > radius_max_)
   {
     PCL_DEBUG ("[pcl::SampleConsensusModelDoubleLine::isModelValid] Radius is too big: should be smaller than %g, but is %g.\n",
                radius_max_, model_coefficients[6]);
     return (false);
   }*/

    return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelDoubleLine(PointT) template class PCL_EXPORTS pcl::SampleConsensusModelDoubleLine<PointT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_DOUBLE_LINE_H_

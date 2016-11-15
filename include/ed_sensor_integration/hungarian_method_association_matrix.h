#ifndef ED_SENSOR_INTEGRATION_HUNGARIAN_METHOD_ASSOCIATION_MATRIX_H_
#define ED_SENSOR_INTEGRATION_HUNGARIAN_METHOD_ASSOCIATION_MATRIX_H_

#include "munkres/munkres.h"
#include <ros/console.h>

namespace ed_sensor_integration
{

class HungarianMethodAssociationMatrix
{

public:

  HungarianMethodAssociationMatrix(size_t num_measurements,
                   size_t num_tracks,
                   double p_not_observed,
                   double p_false_detection,
                   double p_new_track_detection) :
    num_measurements_(num_measurements),
    num_tracks_(num_tracks),
    matrix_size_(num_measurements_ * 2 + num_tracks_)
  {
    assert(num_measurements_ != 0);

    // Resize matrix and set all values to 0
    matrix_.resize(matrix_size_, matrix_size_, 0);

    // Set false detections
    for (unsigned int i = 0; i < num_measurements_; ++i)
    {
      for (unsigned int j = num_tracks_; j < num_tracks_ + num_measurements_; ++j)
      {
        if (i + num_tracks_ == j)
        {
          matrix_(i, j) = p_false_detection;
        }
      }
    }

    // Set new detections
    for (unsigned int i = 0; i < num_measurements_; ++i)
    {
      for (unsigned int j = num_tracks_ + num_measurements_; j < matrix_size_; ++j)
      {
        if (i + num_tracks_ + num_measurements_ == j)
        {
          matrix_(i, j) = p_new_track_detection;
        }
      }
    }

    // Set not observed
    for (unsigned int i = num_measurements_; i < matrix_size_; ++i)
    {
      for (unsigned int j = 0; j < num_tracks_; ++j)
      {
        if (i == j + num_measurements_)
        {
          matrix_(i, j) = p_not_observed;
        }
      }
    }
  }

  ~HungarianMethodAssociationMatrix()
  {

  }

  void setEntry(int i_measurement, int j_track, double p)
  {
    assert(i_measurement < num_measurements_);
    assert(j_track < num_tracks_);

    matrix_(i_measurement, j_track) = p;
  }

  bool solve()
  {
      Munkres<double> m;
      m.solve(matrix_);

      ROS_ERROR_STREAM(matrix_);

      return true;
  }

private:
  size_t num_measurements_, num_tracks_;
  size_t matrix_size_;
  Matrix<double> matrix_;

};

}

#endif

#ifndef ED_SENSOR_INTEGRATION_HUNGARIAN_METHOD_matrix__H_
#define ED_SENSOR_INTEGRATION_HUNGARIAN_METHOD_matrix__H_

#include "munkres/munkres.h"
#include <ros/console.h>

namespace ed_sensor_integration
{

class HungarianMethodAssociationMatrix
{
    const double HIGH_VALUE = 1e9;

public:

  HungarianMethodAssociationMatrix(size_t num_measurements,
                                   size_t num_tracks,
                                   double c_not_observed,
                                   double c_false_detection,
                                   double c_new_track_detection ) :
    num_measurements_(num_measurements),
    num_tracks_(num_tracks),
    matrix_size_(num_measurements_ * 2 + num_tracks_),
    matrix_(matrix_size_, matrix_size_)
  {
    assert(num_measurements_ != 0);

    // Fill association matrix
    // Cost of associating a measurement with a track
    for ( size_t i = 0; i < num_measurements_; i++ )
    {
        for ( size_t j = 0; j < num_tracks_; j++ )
        {
            matrix_(i,j) = 0.0;
        }
    }
    // Cost of a false detection
    for (size_t i = 0; i < num_measurements_; ++i)
    {
        for (size_t j = num_tracks_; j < num_tracks_ + num_measurements_; ++j)
        {
            if (i + num_tracks_ == j)
            {
                matrix_(i, j) = c_false_detection;
            }
            else
            {
                matrix_(i, j) = HIGH_VALUE;
            }
        }
    }
    // Cost of spawning a new track
    for (size_t i = 0; i < num_measurements_; ++i)
    {
        for (size_t j = num_tracks_ + num_measurements_; j < matrix_size_; ++j)
        {
            if (i + num_tracks_ + num_measurements_ == j)
            {
                matrix_(i, j) = c_new_track_detection;
            }
            else
            {
                matrix_(i, j) = HIGH_VALUE;
            }
        }
    }
    // Cost of not associating any measurement to a certain track
    for (size_t i = num_measurements_; i < matrix_size_; ++i)
    {
        for (size_t j = 0; j < num_tracks_; ++j)
        {
            if (i == j + num_measurements_)
            {
                matrix_(i, j) = c_not_observed;
            }
            else
            {
                matrix_(i, j) = HIGH_VALUE;
            }
        }
    }
  }

  ~HungarianMethodAssociationMatrix()
  {

  }

  void setEntry(int i_measurement, int j_track, double cost)
  {
    assert(i_measurement < num_measurements_);
    assert(j_track < num_tracks_);

    matrix_(i_measurement, j_track) = cost;
  }

  std::vector<int> solve()
  {
      // Solve the optimization problem
      Munkres<double> m;
      m.solve(matrix_);

      std::vector<int> associations(num_measurements_,-1);

      // The matrix now contains a zero in every row. The column of the zero indicates the track the measurement is assigned to.
      for ( size_t i = 0 ; i < num_measurements_; i++ )
      {
        for ( size_t j = 0 ; j < num_tracks_; j++  )
        {
          if ( matrix_(i,j) == 0 )
          {
            associations[i] = j;
            break;
          }
        }
      }

      return associations;
  }

private:
  size_t num_measurements_, num_tracks_;
  size_t matrix_size_;
  Matrix<double> matrix_;

};

}

#endif

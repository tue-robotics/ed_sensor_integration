#ifndef ED_SENSOR_INTEGRATION_FITTER_BEAM_MODEL_H_
#define ED_SENSOR_INTEGRATION_FITTER_BEAM_MODEL_H_

#include <geolib/datatypes.h>

class BeamModel
{

public:

    BeamModel();

    BeamModel(double w, unsigned int num_beams);

    void initialize(double w, unsigned int num_beams);

    inline int CalculateBeam(double x, double depth) const
    {
        return (fx_ * x) / depth + half_num_beams_;
    }

    inline geo::Vec2 CalculatePoint(int i, double depth) const
    {
        return rays_[i] * depth;
    }

    inline void CalculatePoints(const std::vector<double>& ranges, std::vector<geo::Vec2>& points)
    {
        static double nan = 0.0 / 0.0;
        points.resize(ranges.size());
        for(unsigned int i = 0; i < points.size(); ++i)
        {
            if (ranges[i] <= 0)
                points[i] = geo::Vec2(nan, nan);
            else
                points[i] = CalculatePoint(i, ranges[i]);
        }
    }

    void RenderModel(const std::vector<std::vector<geo::Vec2> >& contours, const geo::Transform2& pose, int identifier,
                     std::vector<double>& ranges, std::vector<int>& identifiers) const;

    inline unsigned int num_beams() const { return rays_.size(); }

    inline const std::vector<geo::Vec2>& rays() const { return rays_; }

private:

    double fx_;
    unsigned int half_num_beams_;
    std::vector<geo::Vec2> rays_;
};

#endif

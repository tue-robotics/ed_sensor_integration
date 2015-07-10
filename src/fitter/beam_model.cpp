#include "beam_model.h"

// ----------------------------------------------------------------------------------------------------

BeamModel::BeamModel() : half_num_beams_(0) {}

// ----------------------------------------------------------------------------------------------------

BeamModel::BeamModel(double w, unsigned int num_beams)
{
    initialize(w, num_beams);
}

// ----------------------------------------------------------------------------------------------------

void BeamModel::initialize(double w, unsigned int num_beams)
{
    half_num_beams_ = num_beams / 2;
    fx_ = 2 * num_beams / w;

    rays_.resize(num_beams);
    for(unsigned int i = 0; i < num_beams; ++i)
        rays_[i] = geo::Vec2(((double)(i) - half_num_beams_) / fx_, 1);
}

// ----------------------------------------------------------------------------------------------------

void BeamModel::RenderModel(const std::vector<std::vector<geo::Vec2> >& contours, const geo::Transform2& pose, std::vector<double>& ranges) const
{
    for(std::vector<std::vector<geo::Vec2> >::const_iterator it_contour = contours.begin(); it_contour != contours.end(); ++it_contour)
    {
        const std::vector<geo::Vec2>& model = *it_contour;

        std::vector<geo::Vec2> t_vertices(model.size());
        for(unsigned int i = 0; i < model.size(); ++i)
            t_vertices[i] = pose * model[i];

        int nbeams = num_beams();

        for(unsigned int i = 0; i < model.size(); ++i)
        {
            unsigned int j = (i + 1) % model.size();
            const geo::Vec2& p1 = t_vertices[i];
            const geo::Vec2& p2 = t_vertices[j];

            int i1 = CalculateBeam(p1.x, p1.y) + 1;
            int i2 = CalculateBeam(p2.x, p2.y);

            if (i2 < i1 || i2 < 0 || i1 >= nbeams)
                continue;

            i1 = std::max(0, i1);
            i2 = std::min(i2, nbeams - 1);

            geo::Vec2 s = p2 - p1;

            for(int i_beam = i1; i_beam <= i2; ++i_beam)
            {
                const geo::Vec2& r = rays_[i_beam];

                // calculate depth of intersection between line (p1, p2) and r
                double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);

                double& depth_old = ranges[i_beam];
                if (d > 0 && (d < depth_old || depth_old == 0))
                    depth_old = d;
            }
        }
    }
}


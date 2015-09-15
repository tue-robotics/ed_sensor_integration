#include "ed/kinect/beam_model.h"

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

void BeamModel::RenderModel(const std::vector<std::vector<geo::Vec2> >& contours, const geo::Transform2& pose, int identifier,
                            std::vector<double>& ranges, std::vector<int>& identifiers) const
{
    double near_plane = 0.01;

    geo::Vec2 p1_temp, p2_temp;

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
            const geo::Vec2* p1 = &t_vertices[i];
            const geo::Vec2* p2 = &t_vertices[j];

            // If p1 is behind the near plane, clip it
            if (p1->y < near_plane)
            {
                // if p2 is also behind the near plane, skip the line
                if (p2->y < near_plane)
                    continue;

                double r = (near_plane - p1->y) / (p2->y - p1->y);
                p1_temp.x = p1->x + r * (p2->x - p1->x);
                p1_temp.y = near_plane;
                p1 = &p1_temp;
            }

            // If p2 is behind the near plane, clip it
            if (p2->y < near_plane)
            {
                double r = (near_plane - p2->y) / (p1->y - p2->y);
                p2_temp.x = p2->x + r * (p1->x - p2->x);
                p2_temp.y = near_plane;
                p2 = &p2_temp;
            }

            // Calculate the beam numbers corresponding to p1 and p2
            int i1 = CalculateBeam(p1->x, p1->y) + 1;
            int i2 = CalculateBeam(p2->x, p2->y);

            // If i2 < i1, we are looking at the back face of the line, so skip it (back face culling)
            // If i2 < 0 or i1 >= nbeams, the whole line is out of view, so skip it
            if (i2 < i1 || i2 < 0 || i1 >= nbeams)
                continue;

            // Clip i1 and i2 to be between 0 and nbeams
            i1 = std::max(0, i1);
            i2 = std::min(i2, nbeams - 1);

            geo::Vec2 s = *p2 - *p1;
            double t = p1->x * s.y - p1->y * s.x;

            for(int i_beam = i1; i_beam <= i2; ++i_beam)
            {
                const geo::Vec2& r = rays_[i_beam];

                // calculate depth of intersection between line (p1, p2) and r
                double d =  t / (r.x * s.y - r.y * s.x);

                double& depth_old = ranges[i_beam];
                if (d > 0 && (d < depth_old || depth_old == 0))
                {
                    depth_old = d;
                    identifiers[i_beam] = identifier;
                }
            }
        }
    }
}


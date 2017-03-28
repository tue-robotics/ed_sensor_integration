#include "ed/kinect/mesh_tools.h"

#include <geolib/Mesh.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>

namespace dml
{

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p, int d_start, std::vector<geo::Vec2i>& points,
                  std::vector<geo::Vec2i>& line_starts, cv::Mat& contour_map, bool add_first)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p.y, p.x);

    int d_current = d_start; // Current direction
    int x2 = p.x;
    int y2 = p.y;

    int line_piece_min = 1e9; // minimum line piece length of current line
    int line_piece_max = 0; // maximum line piece length of current line

    int d_main = d_current; // The main direction in which we're heading. If we follow a line
                            // that gradually changes to the side (1-cell side steps), this direction
                            // denotes the principle axis of the line

    if (add_first)
        points.push_back(p);

    int n_uninterrupted = 1;
    geo::Vec2i p_corner = p;

    while (true)
    {
        bool found = false;
        int d = (d_current + 3) % 4; // check going left first

        for(int i = -1; i < 3; ++i)
        {
            int idx_x = x2 + dx[d];
            int idx_y = y2 + dy[d];

            // Clip the indices
            idx_x = std::min(image.cols - 1, std::max(0, idx_x));
            idx_y = std::min(image.rows - 1, std::max(0, idx_y));

            if (idx_y < 0 || idx_y >= image.rows || idx_x < 0 || idx_x >= image.cols)
            {
                ROS_ERROR("This should not happen! Image going out of bound, findContours. [%d, %d] and image size is [%d,%d]", idx_y, idx_x, image.rows, image.cols);
                return;
            }
            if (image.at<unsigned char>(idx_y, idx_x) == v)
            {
                found = true;
                break;
            }

            d = (d + 1) % 4;
        }

        if (!found)
            return;

        geo::Vec2i p_current(x2, y2);

        if ((d + 2) % 4 == d_current)
        {
            // 180 degree turn

            if (x2 == p.x && y2 == p.y) // Edge case: if we returned to the start point and
                                        // this is a 180 degree angle, return without adding it
                return;


            points.push_back(p_current);
            d_main = d;
            line_piece_min = 1e9;
            line_piece_max = 0;

        }
        else if (d_current != d_main)
        {
            // Not moving in main direction (side step)

            if (d != d_main)
            {
                // We are not moving back in the main direction
                // Add the corner to the list and make this our main direction

                points.push_back(p_corner);
                d_main = d_current;
                line_piece_min = 1e9;
                line_piece_max = 0;
            }
        }
        else
        {
            // Moving in main direction (no side step)

            if (d_current != d)
            {
                // Turning 90 degrees

                // Check if the length of the last line piece is OK w.r.t. the other pieces in this line. If it differs to much,
                // (i.e., the contour has taken a different angle), add the last corner to the list. This way, we introduce a
                // bend in the contour
                if (line_piece_max > 0 && (n_uninterrupted < line_piece_max - 2 || n_uninterrupted > line_piece_min + 2))
                {
                    // Line is broken, add the corner as bend
                    points.push_back(p_corner);

                    line_piece_min = 1e9;
                    line_piece_max = 0;
                }

                // Update the line piece lenth boundaries with the current found piece
                line_piece_min = std::min(line_piece_min, n_uninterrupted);
                line_piece_max = std::max(line_piece_max, n_uninterrupted);
            }
        }

        if (d_current != d)
        {
            p_corner = p_current;
            n_uninterrupted = 0;
        }

        if ((d_current == 3 && d != 2) || (d == 3 && d != 0)) // up
            line_starts.push_back(p_current);

        if (p_current.y < 0 || p_current.y >= contour_map.rows || p_current.x < 0 || p_current.x >= contour_map.cols)
        {
            ROS_ERROR("This should not happen! Contour map going out of bound, findContours.");
            return;
        }
        contour_map.at<unsigned char>(p_current.y, p_current.x) = 1;

        ++n_uninterrupted;

        if (points.size() > 1 && x2 == p.x && y2 == p.y)
            return;

        x2 = x2 + dx[d];
        y2 = y2 + dy[d];

        d_current = d;
    }
}

// ----------------------------------------------------------------------------------------------------

void calculateContour(const cv::Mat& image, std::vector<std::vector<geo::Vec2i> >& contours)
{
    cv::Mat contour_map(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char v = image.at<unsigned char>(y, x);

            if (v == 0)
                continue;

            contours.push_back(std::vector<geo::Vec2i>());
            std::vector<geo::Vec2i>& points = contours.back();

            std::vector<geo::Vec2i> line_starts;
            findContours(image, geo::Vec2i(x, y), 0, points, line_starts, contour_map, true);

            unsigned int num_points = points.size();

            if (num_points <= 2)
            {
                contours.pop_back();
                continue;
            }

            // Check for holes within the contour
            for(unsigned int i = 0; i < line_starts.size(); ++i)
            {
                int x2 = line_starts[i].x;
                int y2 = line_starts[i].y;

                while(image.at<unsigned char>(y2, x2) == v)
                    ++x2;

                if (contour_map.at<unsigned char>(y2, x2 - 1) != 0)
                    continue;

                // found a hole, so find the contours of this hole

                contours.push_back(std::vector<geo::Vec2i>());
                std::vector<geo::Vec2i>& hole_points = contours.back();

                findContours(image, geo::Vec2i(x2 - 1, y2 + 1), 1, hole_points, line_starts, contour_map, false);

                if (hole_points.size() <= 2)
                {
                    contours.pop_back();
                    continue;
                }
            }

            // Remove the shape
            cv::floodFill(image, cv::Point(x, y), 0);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void project2D(const geo::Mesh& mesh, std::vector<std::vector<geo::Vec2> >& contours)
{
    const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
    const std::vector<geo::Vec3>& vertices = mesh.getPoints();

    if (vertices.empty())
        return;

    // Calculate bounding rectangle

    geo::Vec2 min(vertices[0].x, vertices[0].y);
    geo::Vec2 max = min;

    for(std::vector<geo::Vec3>::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
    {
        const geo::Vec3& v = *it;
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
    }

    // Initialize grid
    double w = max.x - min.x;
    double h = max.y - min.y;
    double res = 500 / std::min(w, h);     // TODO: get rid of ad-hoc resolution calculation
    cv::Mat grid(h * res + 5, w * res + 5, CV_8UC1, cv::Scalar(0));

    // Downproject vertices
    std::vector<cv::Point2d> proj_vertices(vertices.size());
    for(unsigned int i = 0; i < proj_vertices.size(); ++i)
    {
        const geo::Vec3& v = vertices[i];
        proj_vertices[i] = cv::Point2d((v.x - min.x) * res + 1, (v.y - min.y) * res + 1);
    }

    // Draw triangles
    std::vector<cv::Point> pts(3);
    for(std::vector<geo::TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        const geo::TriangleI& t = *it;
        pts[0] = proj_vertices[t.i1_];
        pts[1] = proj_vertices[t.i2_];
        pts[2] = proj_vertices[t.i3_];
        cv::fillConvexPoly(grid, &pts[0], 3, cv::Scalar(1));
    }

//    cv::Mat canvas = grid.clone() * 255;

    // Calculate contours
    std::vector<std::vector<geo::Vec2i> > contours_pixel;
    calculateContour(grid, contours_pixel);

    contours.resize(contours_pixel.size());
    for(unsigned int i = 0; i < contours_pixel.size(); ++i)
    {
        std::vector<geo::Vec2i>& contour_pixel = contours_pixel[i];
        std::vector<geo::Vec2>& contour = contours[i];
        contour.resize(contour_pixel.size());

        for(unsigned int j = 0; j < contour.size(); ++j)
            contour[j] = geo::Vec2(contour_pixel[j].x - 1, contour_pixel[j].y - 1) / res + min;
    }


//    for(std::vector<std::vector<geo::Vec2i> >::const_iterator it = contours_pixel.begin(); it != contours_pixel.end(); ++it)
//    {
//        const std::vector<geo::Vec2i>& contour = *it;
//        for(unsigned int i = 0; i < contour.size(); ++i)
//        {
//            unsigned int j = (i + 1) % contour.size();

//            const geo::Vec2i& p1 = contour[i];
//            const geo::Vec2i& p2 = contour[j];

//            cv::line(canvas, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(128), 2);
//        }

//        for(std::vector<geo::Vec2i>::const_iterator it2 = it->begin(); it2 != it->end(); ++it2)
//        {
//            const geo::Vec2i& p = *it2;
//            cv::circle(canvas, cv::Point(p.x, p.y), 5, cv::Scalar(128));
//        }
//    }

//    cv::imshow("grid", canvas);
//    cv::waitKey();

}

} // end namespace dml


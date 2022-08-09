#ifndef ED_SENSOR_INTEGRATION_TOOLS_FITTER_VIZ_H_
#define ED_SENSOR_INTEGRATION_TOOLS_FITTER_VIZ_H_

#include <ed/entity.h>
#include <ed/world_model.h>

#include <ed/kinect/fitter.h>

#include <geolib/math_types.h>

#include <opencv2/highgui/highgui.hpp>


// visualization parameters
int canvas_width = 800; // pixels
int canvas_height = 600; // pixels
float canvas_resolution = 100; // pixels per meter

int sensor_x = canvas_width/2;
int sensor_y = canvas_height * 8/10;
cv::Point sensorlocation(sensor_x, sensor_y);

cv::Scalar sensor_colour(0, 255, 0); // green
cv::Scalar entity_colour(0, 255, 255); // yellow
cv::Scalar fitted_colour(243, 192, 15); // blue
cv::Scalar measurement_colour(161, 17, 187); // purple

/**
 * @brief drawLine, draw a line in 2D metric space on a canvas
 * @param canvas: canvas to draw the line to
 * @param point1: first point defining the line in map frame
 * @param point2: second point defining the line in map frame
 * @param pose: position of the sensor in map frame.
 * @param resolution: pixels per meter
 * @param origin_x: location in pixels of the sensor on the canvas
 * @param origin_y: location in pixels of the sensor on the canvas
 * @param colour: colour to draw the line
 */
void drawLine(const cv::Mat& canvas, geo::Vec2 point1, geo::Vec2 point2, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar colour)
{
    // computing points relative to the pose (in meters)
    geo::Vec2 rel_point1 = pose * point1;
    geo::Vec2 rel_point2 = pose * point2;

    // position to pixels
    int x_p1 = origin_x + floor(rel_point1.x * resolution);
    int y_p1 = origin_y - floor(rel_point1.y * resolution);
    int x_p2 = origin_x + floor(rel_point2.x * resolution);
    int y_p2 = origin_y - floor(rel_point2.y * resolution);

    if (x_p1 < 0 || x_p2 < 0|| x_p1 >= canvas.cols || x_p2 >= canvas.cols)
    {
        std::cout << "Entity: x-coordinate out of range" << std::endl;
        return;
    }
    if (y_p1 < 0 || y_p2 < 0|| y_p1 >= canvas.rows || y_p2 >= canvas.rows)
    {
        std::cout << "Entity: y-coordinate out of range" << std::endl;
        return;
    }

    // paint to screen
    cv::Point point1_p(x_p1, y_p1);
    cv::Point point2_p(x_p2, y_p2);
    cv::line(canvas, point1_p, point2_p, colour, 1);
}

/**
 * @brief drawShape2D draw a 2D shape defined in 2D metric space on a canvas.
 * @param canvas: canvas to draw to
 * @param shape: 2D shape to draw
 * @param pose: pose in meters of the center of the shape.
 * @param resolution: pixels per meter
 * @param origin_x: postion of the origin in pixels
 * @param origin_y: position of the origin in pixels
 * @param colour: colour to draw the shape in.
 */
void drawShape2D(const cv::Mat& canvas, const Shape2D& shape, geo::Transform2 pose, float resolution, int origin_x, int origin_y, cv::Scalar colour)
{
    for (unsigned int i=0; i < shape.size(); i++)
    {
        for (unsigned int j=0; j < shape[i].size()-1; j++)
        {
            drawLine(canvas, shape[i][j], shape[i][j+1], pose, resolution, origin_x, origin_y, colour);
        }
        drawLine(canvas, shape[i][0], shape[i][shape[0].size()-1], pose, resolution, origin_x, origin_y, colour);
    }

    // paint entity center

    int x_p_ent = origin_x + floor(pose.t.x * resolution);
    int y_p_ent = origin_y - floor(pose.t.y * resolution);

    cv::Point Entity_center(x_p_ent, y_p_ent);
    cv::circle(canvas, Entity_center, 3, colour, cv::FILLED);
}

/**
 * create an image displaying the fitted entity along with the information used in fitting.
 *
 * Displays the fitted entity, sensor data used in fitting and the original
 * pose of the entity prior to fitting.
 *
 * Note that sensor_pose,
 * entity_pose and fitted_pose must be expressed in the same coordinate frame
 * @param entity: shape of the entity that was fitted
 * @param sensor_pose: pose of the sensor
 * @param entity_pose: estimated pose of the entity prior to fitting
 * @param fitted_pose: estimated pose of the entity after fitting
 * @param fitterdata: fitterdata used in fitting the entity.
 * @param estimateEntityPose: whether or not the fitting encountered errors
 * @return
 */
cv::Mat visualizeFitting(EntityRepresentation2D entity, geo::Transform2 sensor_pose, geo::Transform2 entity_pose,
                         geo::Transform2 fitted_pose, FitterData fitterdata, bool estimateEntityPose)
{
    // visualize fitting
    cv::Mat canvas = cv::Mat(canvas_height, canvas_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // paint to screen the location of the sensor
    cv::circle(canvas, sensorlocation, 3, sensor_colour, cv::FILLED);

    // paint entity (from worldmodel)
    geo::Transform2 relpose = sensor_pose.inverse() * entity_pose;
    drawShape2D(canvas, entity.shape_2d, relpose, canvas_resolution, sensor_x, sensor_y, entity_colour);

    if (estimateEntityPose)
    {
        // paint fitted entity
        geo::Transform2 fitted_relpose = sensor_pose.inverse() * fitted_pose;
        drawShape2D(canvas, entity.shape_2d, fitted_relpose, canvas_resolution, sensor_x, sensor_y, fitted_colour);
    }
    else
    {
        std::cout << "Fitted entity not printed in image because of a fitter error" << std::endl;
    }

    // paint sensor_ranges
    uint nranges = fitterdata.sensor_ranges.size();
    uint half_nranges = nranges/2;
    float fx = fitterdata.fx;
    for (unsigned int i = 0; i < nranges; ++i)
    {
        float x_m = fitterdata.sensor_ranges[i] * ((static_cast< float >(i) - half_nranges) / fx);
        float y_m = fitterdata.sensor_ranges[i];

        // postion to pixels
        int x_p = sensor_x + floor(x_m * canvas_resolution);
        int y_p = sensor_y - floor(y_m * canvas_resolution);

        if (x_p < 0 || x_p >= canvas_width || y_p < 0 || y_p >= canvas_height)
            continue;
        if (fitterdata.sensor_ranges[i] == 0) // filter out sensor_ranges equal to zero
            continue;

        // paint to screen
        cv::Point centerCircle(x_p, y_p);
        cv::circle(canvas, centerCircle, 2, measurement_colour, cv::FILLED);
    }

    return canvas;
}
#endif // ED_SENSOR_INTEGRATION_TOOLS_FITTER_VIZ_H_

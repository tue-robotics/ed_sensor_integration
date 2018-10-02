#include "plugin_tracking.h"

#include <iostream>

#include <ros/node_handle.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
//#include <ed/helpers/depth_data_processing.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ed/convex_hull_calc.h"

#include <ed/io/json_writer.h>

#include "ed_sensor_integration/association_matrix.h"

#include <tue/profiling/timer.h>
#include <numeric>
#include <cmath>
#include <iterator>

namespace
{

typedef std::vector<unsigned int> ScanSegment;



struct EntityUpdate
{
    ed::ConvexHull chull;
    geo::Pose3D pose;
    std::string flag; // Temp for RoboCup 2015; todo: remove after
};

visualization_msgs::Marker getMarker ( ed::tracking::FeatureProperties& featureProp, int ID, bool possiblyMobidik ) // TODO move to ed_rviz_plugins?
{
    visualization_msgs::Marker marker;
    std_msgs::ColorRGBA color;

    if ( possiblyMobidik )
    {
        color.r = 0;
        color.g = 0;
        color.b = 1;
        color.a = ( float ) 0.5;
        
        ed::tracking::Rectangle rectangle = featureProp.getRectangle();
        rectangle.setMarker ( marker , ID, color, "Mobidik" );
    }
    else
    {
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = ( float ) 0.5;

        if ( featureProp.getFeatureProbabilities().get_pCircle() > featureProp.getFeatureProbabilities().get_pRectangle() )
        {
            ed::tracking::Circle circle = featureProp.getCircle();
            circle.setMarker ( marker , ID, color );
        }
        else
        {
            ed::tracking::Rectangle rectangle = featureProp.getRectangle();
            rectangle.setMarker ( marker , ID, color );
        }
    }
    
    return marker;
}


// ----------------------------------------------------------------------------------------------------

double getFittingError(const ed::Entity& e, const geo::LaserRangeFinder& lrf, const geo::Pose3D& rel_pose,
                       const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                       int& num_model_points)
{
    std::vector<double> test_model_ranges = model_ranges;

    // Render the entity with the given relative pose
    geo::LaserRangeFinder::RenderOptions opt;
    opt.setMesh(e.shape()->getMesh(), rel_pose);

    geo::LaserRangeFinder::RenderResult res(test_model_ranges);
    lrf.render(opt, res);

    int n = 0;
    num_model_points = 0;
    double total_error = 0;
    for(unsigned int i = 0; i < test_model_ranges.size(); ++i)
    {
        double ds = sensor_ranges[i];
        double dm = test_model_ranges[i];

        if (ds <= 0)
            continue;

        ++n;

        if (dm <= 0)
        {
            total_error += 0.1;
            continue;
        }

        double diff = std::abs(ds - dm);
        if (diff < 0.1)
            total_error += diff;
        else
        {
            if (ds > dm)
                total_error += 1;
            else
                total_error += 0.1;
        }

        ++num_model_points;




//        if (dm <= 0 && model_ranges[i]>0)
//        {
//            total_error += 0.3;
//            continue;
//        }

//        if (dm < model_ranges[i])
//            ++num_model_points;
//        else if(model_ranges[i]<=0 && dm>0 && dm<=7) //when there is no world model behind the door, output of world render without door is zero.
//        {                                              //only taking points with no world behind it, into account when nearby.
//            ++num_model_points;
//        }                                              // giving problems with fitting when door is not good in view

//        double diff = std::abs(ds - dm);
//        if (diff < 0.3)
//            total_error += diff;
//        else
//        {
//            if (ds > dm)
//                total_error += 1;
//            else
//                total_error += 0.3;
//        }
    }

    return total_error / (n+1); // to be sure to never divide by zero.
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D getPoseFromCache(const ed::Entity& e, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const ed::UUID ID = e.id();
    geo::Pose3D old_pose = e.pose();
    if (pose_cache.find(ID) == pose_cache.end())
    {
        pose_cache[ID] = old_pose;
    }
    else
    {
        old_pose = pose_cache[ID];
    }
    return old_pose;
}

// ----------------------------------------------------------------------------------------------------

geo::Pose3D fitEntity(const ed::Entity& e, const geo::Pose3D& sensor_pose, const geo::LaserRangeFinder& lrf,
                      const std::vector<float>& sensor_ranges, const std::vector<double>& model_ranges,
                      float x_window, float x_step, float y_window, float y_step, float yaw_min, float yaw_plus, float yaw_step, std::map<ed::UUID,geo::Pose3D>& pose_cache)
{
    const geo::Pose3D& old_pose = getPoseFromCache(e, pose_cache);

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();


    double min_error = 1e6;
    geo::Pose3D best_pose = e.pose();


    for(float dyaw = yaw_min; dyaw <= yaw_plus; dyaw += yaw_step)
    {
        geo::Mat3 rot;
        rot.setRPY(0, 0, dyaw);
        geo::Pose3D test_pose = old_pose;
        test_pose.R = old_pose.R * rot;

        for(float dx = -x_window; dx <= x_window; dx += x_step)
        {
            test_pose.t.x = old_pose.t.x + dx;
            for(float dy = -y_window; dy <= y_window; dy += y_step)
            {
                test_pose.t.y = old_pose.t.y + dy;

                int num_model_points;
                double error = getFittingError(e, lrf, sensor_pose_inv * test_pose, sensor_ranges, model_ranges, num_model_points);

//                ROS_ERROR_STREAM("yaw = " << dyaw << ", error = " << error << ", minerror= " << min_error << ", num_model_points = " << num_model_points);

                if (error < min_error && num_model_points >= 3)
                {
                    best_pose = test_pose;
                    min_error = error;
                }
            }
        }
    }
    //    if (best_pose != e.pose())
    //        std::cout<<"new_pose"<<std::endl;
    return best_pose;
}


// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(double x_sensor, double y_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    int i_beam = lrf.getAngleUpperIndex(x_sensor, y_sensor);
    if (i_beam < 0 || i_beam >= sensor_ranges.size())
        return true; // or actually, we don't know

    float rs = sensor_ranges[i_beam];
    return rs == 0 || geo::Vec2(x_sensor, y_sensor).length() > rs - 0.1;
}

// ----------------------------------------------------------------------------------------------------

bool pointIsPresent(const geo::Vector3& p_sensor, const geo::LaserRangeFinder& lrf, const std::vector<float>& sensor_ranges)
{
    return pointIsPresent(p_sensor.x, p_sensor.y, lrf, sensor_ranges);
}

}

// ----------------------------------------------------------------------------------------------------

// Strongly inspired by https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
template<typename T>
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment( T& p, T& q, T& r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}
 
 template<typename T>
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation( T& p, T& q, T& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
 template<typename T>
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect( T& p1, T& q1, T& p2, T& q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}
 
 template<typename T>
// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(std::vector<T> Points, T& p)
{
     int n = Points.size();   
        
    // There must be at least 3 vertices in polygon[]
    if (n < 3)  return false;
    
    // Create a point for line segment from p to infinite
    T extreme;
    extreme.x = INF;
    extreme.y = p.y;
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(Points[i], Points[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(Points[i], p, Points[next]) == 0)
               return onSegment(Points[i], p, Points[next]);
            count++;
        }
        i = next;
    } while (i != 0);
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}


// ----------------------------------------------------------------------------------------------------

LaserPluginTracking::LaserPluginTracking() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LaserPluginTracking::~LaserPluginTracking()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string laser_topic;
    config.value("laser_topic", laser_topic);
    config.value("world_association_distance", world_association_distance_);
    config.value("min_segment_size_pixels", min_segment_size_pixels_);
    config.value("segment_depth_threshold", segment_depth_threshold_);
    config.value("min_cluster_size", min_cluster_size_);
    config.value("max_cluster_size", max_cluster_size_);
    config.value("max_gap_size", max_gap_size_);
    config.value("nominal_corridor_width", nominal_corridor_width_);
    

    int i_fit_entities = 0;
    config.value("fit_entities", i_fit_entities, tue::OPTIONAL);
    fit_entities_ = (i_fit_entities != 0);
    
    int i_check_door_status = 0;
    config.value("check_door_status", i_check_door_status, tue::OPTIONAL);
    check_door_status_ = (i_check_door_status != 0);

    if (config.hasError())
        return;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&cb_queue_);

    // Communication
    sub_scan_ = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 3, &LaserPluginTracking::scanCallback, this);
    door_pub_ = nh.advertise<ed_sensor_integration::doorDetection>("door", 3);
    ObjectMarkers_pub_ = nh.advertise<visualization_msgs::MarkerArray> ( "ed/gui/objectMarkers", 3 );

    tf_listener_ = new tf::TransformListener;

    //pose_cache.clear();
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    cb_queue_.callAvailable();

    while(!scan_buffer_.empty())
    {
        sensor_msgs::LaserScan::ConstPtr scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
            scan_buffer_.pop();
            geo::Pose3D sensor_pose;
            geo::convert(t_sensor_pose, sensor_pose);
            update(world, scan, sensor_pose, req);
        }
        catch(tf::ExtrapolationException& ex)
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << ex.what());
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
                {
                    // Scan is too new
                    break;
                }
                else
                {
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer_.pop();
                }
            }
            catch(tf::TransformException& exc)
            {
                scan_buffer_.pop();
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << exc.what());
            scan_buffer_.pop();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserPluginTracking::update(const ed::WorldModel& world, const sensor_msgs::LaserScan::ConstPtr& scan,
                         const geo::Pose3D& sensor_pose, ed::UpdateRequest& req)
{
    tue::Timer t_total;
    t_total.start();

    // - - - - - - - - - - - - - - - - - -
    // Update laser model

    std::vector<float> sensor_ranges(scan->ranges.size());
    for(unsigned int i = 0; i < scan->ranges.size(); ++i)
    {
        float r = scan->ranges[i];
        if (r > scan->range_max)
            sensor_ranges[i] = r;
        else if (r == r && r > scan->range_min)
            sensor_ranges[i] = r;
        else
            sensor_ranges[i] = 0;
    }

    unsigned int num_beams = sensor_ranges.size();

    if (lrf_model_.getNumBeams() != num_beams)
    {
        lrf_model_.setNumBeams(num_beams);
        lrf_model_.setAngleLimits(scan->angle_min, scan->angle_max);
        lrf_model_.setRangeLimits(scan->range_min, scan->range_max);
    }

    // - - - - - - - - - - - - - - - - - -
    // Filter laser data (get rid of ghost points)

    for(unsigned int i = 1; i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];
        // Get rid of points that are isolated from their neighbours
        if (std::abs(rs - sensor_ranges[i - 1]) > 0.1 && std::abs(rs - sensor_ranges[i + 1]) > 0.1)  // TODO: magic number
        {
            sensor_ranges[i] = sensor_ranges[i - 1];
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Render world model as if seen by laser

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    std::vector<double> model_ranges(num_beams, 0);
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
	
        if (e->shape() && e->has_pose() && !(e->hasType("left_door") || e->hasType("door_left") || e->hasType("right_door") || e->hasType("door_right" ) || e->hasFlag("non-localizable")))
        {
	//  std::cout << "Shape after = " << e->shape() << std::endl;
            // Set render options
            geo::LaserRangeFinder::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * e->pose());

            geo::LaserRangeFinder::RenderResult res(model_ranges);
            lrf_model_.render(opt, res);
        }        
    }

    // - - - - - - - - - - - - - - - - - -
    // Fit the doors

    if (fit_entities_)
    {

        for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (!e->shape() || !e->has_pose())
                continue;

            geo::Pose3D e_pose_SENSOR = sensor_pose_inv * e->pose();

            // If not in sensor view, continue
            if (e_pose_SENSOR.t.length2() > 5.0 * 5.0 || e_pose_SENSOR.t.x < 0)
                continue;

            if (e->hasType("left_door") || e->hasType("door_left"))
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity(*e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache);
                req.setPose(e->id(), new_pose);
                //std::cout << "left_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * new_pose);

                geo::LaserRangeFinder::RenderResult res(model_ranges);
                lrf_model_.render(opt, res);
            }
            else if (e->hasType("right_door") || e->hasType("door_right"))
            {
                // Try to update the pose
                geo::Pose3D new_pose = fitEntity(*e, sensor_pose, lrf_model_, sensor_ranges, model_ranges, 0, 0.1, 0, 0.1, -1.57, 1.57, 0.1, pose_cache);
                req.setPose(e->id(), new_pose);
                //std::cout << "right_door" << std::endl;

                // Render the door with the updated pose
                geo::LaserRangeFinder::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), sensor_pose_inv * new_pose);

                geo::LaserRangeFinder::RenderResult res(model_ranges);
                lrf_model_.render(opt, res);
            }
        }
    }

    // - - - - - - - - - - - - - - - - - -
    // Try to associate sensor laser points to rendered model points, and filter out the associated ones

    for(unsigned int i = 0; i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];
        float rm = model_ranges[i];

        if (rs <= 0
                || (rm > 0 && rs > rm)  // If the sensor point is behind the world model, skip it
                || (std::abs(rm - rs) < world_association_distance_))
            sensor_ranges[i] = 0;
    }

    // - - - - - - - - - - - - - - - - - -
    // Segment the remaining points into clusters

    std::vector<ScanSegment> segments;

    // Find first valid value
    ScanSegment current_segment;
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        if (sensor_ranges[i] > 0)
        {
            current_segment.push_back(i);
            break;
        }
    }

    if (current_segment.empty())
    {
        return;
    }

    int gap_size = 0;

    for(unsigned int i = current_segment.front(); i < num_beams - 1; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_)
        {
            // Found a gap
            ++gap_size;

            if (gap_size >= max_gap_size_)
            {
                i = current_segment.back() + 1;

                if (current_segment.size() >= min_segment_size_pixels_)
                {
                    // calculate bounding box
                    geo::Vec2 seg_min, seg_max;
                    for(unsigned int k = 0; k < current_segment.size(); ++k)
                    {
                        geo::Vector3 p = lrf_model_.rayDirections()[current_segment[k]] * sensor_ranges[current_segment[k]];

                        if (k == 0)
                        {
                            seg_min = geo::Vec2(p.x, p.y);
                            seg_max = geo::Vec2(p.x, p.y);
                        }
                        else
                        {
                            seg_min.x = std::min(p.x, seg_min.x);
                            seg_min.y = std::min(p.y, seg_min.y);
                            seg_max.x = std::max(p.x, seg_max.x);
                            seg_max.y = std::max(p.y, seg_max.y);
                        }
                    }

                    geo::Vec2 bb = seg_max - seg_min;
                    if ((bb.x > min_cluster_size_ || bb.y > min_cluster_size_) && bb.x < max_cluster_size_ && bb.y < max_cluster_size_)
                        segments.push_back(current_segment);
                }

                current_segment.clear();

                // Find next good value
                while(sensor_ranges[i] == 0 && i < num_beams)
                    ++i;

                current_segment.push_back(i);
            }
        }
        else
        {
            gap_size = 0;
            current_segment.push_back(i);
        }
    }
    

    std::vector<EntityUpdate> clusters;
    std::vector<ed::tracking::FeatureProperties> measuredProperties;

    for ( std::vector<ScanSegment>::const_iterator it = segments.begin(); it != segments.end(); ++it )
    {
        const ScanSegment& segment = *it;
        std::vector<geo::Vec2f> points ( segment.size() );

        for ( unsigned int i = 0; i < segment.size(); ++i )
        {
            unsigned int j = segment[i];
            geo::Vector3 p_sensor = lrf_model_.rayDirections() [j] * sensor_ranges[j];

            // Transform to world frame
            geo::Vector3 p = sensor_pose * p_sensor;

            // Add to cv array
            points[i] = geo::Vec2f ( p.x, p.y );
        }
        
        std::vector<unsigned int> cornerIndices;
        std::vector<geo::Vec2f>::iterator it_start = points.begin();
        std::vector<geo::Vec2f>::iterator it_end = points.end();
        unsigned int cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
    
        if( ed::tracking::findPossibleCorner ( points, &cornerIndices, &it_start, &it_end ) )
        {
                cornerIndex = cornerIndices[0];
        }
        
        ed::tracking::Circle circle;
        ed::tracking::Rectangle rectangle;
        std::vector<geo::Vec2f>::iterator it_low, it_high;
        

        ed::tracking::FITTINGMETHOD method = ed::tracking::CIRCLE;
        float error_circle2 = ed::tracking::fitObject ( points, method, &cornerIndex, &rectangle, &circle, &it_low, &it_high, sensor_pose );

        method = ed::tracking::determineCase ( points, &cornerIndex, &it_low, &it_high, sensor_pose ); // chose to fit a single line or a rectangle (2 lines)
        float error_rectangle2 = ed::tracking::fitObject ( points, method,  &cornerIndex, &rectangle, &circle, &it_low, &it_high,  sensor_pose );

        ed::tracking::FeatureProbabilities prob;
        prob.setMeasurementProbabilities ( error_rectangle2, error_circle2, 2*circle.get_R(), nominal_corridor_width_ );

        ed::tracking::FeatureProperties properties;
        properties.setFeatureProbabilities ( prob );
        properties.setCircle ( circle );
        properties.setRectangle ( rectangle );

        measuredProperties.push_back ( properties );        
    }
            
    visualization_msgs::MarkerArray markerArray;
    
    // Publish the fitted segments on the ObjectMarkers_pub_ -topic.
    for ( int ii = 0; ii < measuredProperties.size(); ii++ )
    {
        ed::tracking::FeatureProperties property = measuredProperties[ii];
        bool possiblyMobidik = false;
        if ( property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle() && // Dimension check
                property.rectangle_.get_d() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                property.rectangle_.get_w() < MOBIDIK_WIDTH + MOBIDIK_MARGIN &&
                ( property.rectangle_.get_d() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ||
                  property.rectangle_.get_w() > MOBIDIK_WIDTH - MOBIDIK_MARGIN ) )
        {

            for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
            {
                const ed::EntityConstPtr& e = *it;

                std::string MobiDikWaitingAreaID = "MobidikArea";

                if ( e->id().str().length() < MobiDikWaitingAreaID.length() )
                {
                    continue;
                }

                if ( e->id().str().substr ( 0, MobiDikWaitingAreaID.length() ) == MobiDikWaitingAreaID )
                {
                        // It is assumed here that there is a navigation task, so only points on the ground are taken into consideration
                        
                        std::vector<geo::Vector3> points = e.get()->shape().get()->getMesh().getPoints();
                        std::vector<geo::Vector3> groundPoints;
                        const geo::Vec3T<double> pose = e.get()->pose().getOrigin();
                        
                        for(unsigned int iPoints = 0; iPoints < points.size(); iPoints++)
                        {
                                if(points[iPoints].getZ() == 0)
                                {
                                        groundPoints.push_back( points[iPoints] + pose );
                                }
                        }        
                       geo::Vector3 mobidikPoint( property.getRectangle().get_x(), property.getRectangle().get_y(), property.getRectangle().get_z() );
                       
                    if( isInside( groundPoints, mobidikPoint) )
                    {
                        possiblyMobidik = true;
                    }
                }   
            }
        }
        
        visualization_msgs::Marker marker = getMarker ( property, ii, possiblyMobidik ); // TODO make an entity within ED of the object (correct data-association!!) and do a query via a plugin if objects of a certain type are required
        markerArray.markers.push_back( marker );
        
    }
    ObjectMarkers_pub_.publish( markerArray );
}

// ----------------------------------------------------------------------------------------------------


void LaserPluginTracking::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    scan_buffer_.push(msg);
}


ED_REGISTER_PLUGIN(LaserPluginTracking)

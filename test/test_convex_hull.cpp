#include <ed/convex_hull.h>
#include <ed/convex_hull_calc.h>

int main(int argc, char **argv)
{
    ed::ConvexHull c1;
    c1.z_min = -0.5;
    c1.z_max = 0.5;

    c1.points.push_back(geo::Vec2f(0.0640089, 0.0461254));
    c1.points.push_back(geo::Vec2f(0.0315182, 0.050766));
    c1.points.push_back(geo::Vec2f(0.0154809, 0.0444589));
    c1.points.push_back(geo::Vec2f(-0.0493098, -0.00545931));
    c1.points.push_back(geo::Vec2f(-0.050205, -0.0308552));
    c1.points.push_back(geo::Vec2f(-0.0349846, -0.0335279));
    c1.points.push_back(geo::Vec2f(-0.0346048, -0.0335526));
    c1.points.push_back(geo::Vec2f(0.0120594, -0.0250401));
    c1.points.push_back(geo::Vec2f(0.0460369, -0.0129151));


    ed::ConvexHull c2;
    c2.z_min = -0.5;
    c2.z_max = 0.5;

    c2.points.push_back(geo::Vec2f(0.0868937, 0.149448));
    c2.points.push_back(geo::Vec2f(0.0662393, 0.152538));
    c2.points.push_back(geo::Vec2f(0.0023579, 0.162071));
    c2.points.push_back(geo::Vec2f(-0.0376225, 0.158206));
    c2.points.push_back(geo::Vec2f(-0.0583535, 0.151631));
    c2.points.push_back(geo::Vec2f(-0.0845235, -0.0181588));
    c2.points.push_back(geo::Vec2f(-0.0863414, -0.0758137));
    c2.points.push_back(geo::Vec2f(-0.0698329, -0.167838));
    c2.points.push_back(geo::Vec2f(-0.0300491, -0.17906));
    c2.points.push_back(geo::Vec2f(0.0547022, -0.161849));
    c2.points.push_back(geo::Vec2f(0.0759892, -0.146899));
    c2.points.push_back(geo::Vec2f(0.0805406, -0.024275));


    ed::convex_hull::calculateEdgesAndNormals(c1);
    ed::convex_hull::calculateEdgesAndNormals(c2);

    std::cout << ed::convex_hull::collide(c1, geo::Vector3(3.14171, 5.07719, 0), c2, geo::Vector3(3.42953, 3.63333, 0), 0.1, 0.1) << std::endl;

    return 0;
}

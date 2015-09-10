#ifndef DML_MESH_TOOLS_H_
#define DML_MESH_TOOLS_H_

#include <geolib/datatypes.h>

namespace geo
{
class Mesh;
}

namespace dml
{

// Projects mesh down (along z-axis) and generates 2D contour
void project2D(const geo::Mesh& mesh, std::vector<std::vector<geo::Vec2> >& contours);

} // end namespace dml

#endif

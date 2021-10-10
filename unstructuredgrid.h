#ifndef UNSTRUCTUREDGRID_H
#define UNSTRUCTUREDGRID_H

#include <griddatatypes.h>

class UnstructuredGrid
{
public:
    Nodes nodes_;
    Edges edges_;
    Triangles surface_triangles_;
    Tetrahedrons tetrahedrons_;

    Nodes& get_nodes();
    Edges& get_edges();
    Triangles& get_triangles();
    Tetrahedrons& get_tetrahedrons();
};

std::ostream& operator<<(std::ostream& stream, UnstructuredGrid& grid);

#endif // UNSTRUCTUREDGRID_H

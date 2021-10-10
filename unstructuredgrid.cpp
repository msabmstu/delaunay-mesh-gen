#include <unstructuredgrid.h>

Nodes& UnstructuredGrid::get_nodes() {
    return nodes_;
}

Edges& UnstructuredGrid::get_edges() {
    return  edges_;
}

Triangles& UnstructuredGrid::get_triangles() {
    return surface_triangles_;
}

Tetrahedrons& UnstructuredGrid::get_tetrahedrons() {
    return tetrahedrons_;
}

std::ostream& operator<<(std::ostream& stream, UnstructuredGrid& grid) {
    for (auto& n : grid.get_nodes())
        stream << "Node ID: " << n.first << "\tNode coordinates: " << n.second << std::endl;

    for (auto& e : grid.get_edges())
        stream << "Edge ID: " << e.first << "\tPoint IDs: " << e.second << std::endl;

    for (auto& t : grid.get_triangles())
        stream << "Triangle ID: " << t.first << "\tPoint IDs: " << t.second << std::endl;

    for (auto& t : grid.get_tetrahedrons())
        stream << "Tetrahedron ID: " << t.first << "\tPoint IDs: " << t.second << std::endl;

    return stream;
}


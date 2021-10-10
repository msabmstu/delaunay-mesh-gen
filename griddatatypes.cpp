#include <griddatatypes.h>

std::ostream& operator<<(std::ostream& stream, BaseNode& base_node) {
    for (auto& c : base_node.coords_)
        stream << c << ' ';
    stream << std::endl;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, BaseEdge& base_edge) {
    for (auto& id : base_edge.node_ids_)
        stream << id << ' ';
    stream << std::endl;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, BaseTriangle& base_triangle) {
    for (auto& id : base_triangle.node_ids_)
        stream << id << ' ';
    stream << std::endl;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, BaseTetrahedron& base_tetrahedron) {
    for (auto& id : base_tetrahedron.node_ids_)
        stream << id << ' ';
    stream << std::endl;
    return stream;
}

void DelaunayCore3D::clear(){
    tetrahedrons_to_delete_.clear();
    core_front_.clear();
}


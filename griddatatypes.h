#ifndef GRIDDATATYPES_H
#define GRIDDATATYPES_H

#include <iostream>
#include <array>
#include <queue>
#include <tuple>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <set>

struct BaseNode
{
public:
    std::array<double, 3> coords_;

    explicit BaseNode() {
        coords_ = std::array<double, 3>();
    }
    explicit BaseNode(std::array<double, 3> arr) {
            coords_ = arr;
    }
    /*explicit BaseNode(double a, double b, double c) {
        coords_[0] = a;
        coords_[1] = b;
        coords_[2] = c;
    }*/
};

struct BaseEdge
{
public:
    std::array<uint32_t, 2> node_ids_;
    int16_t physical_;
    int16_t elementary_;

    explicit BaseEdge(uint32_t id1 = 0.,
                      uint32_t id2 = 0.,
                      int16_t physical = 0,
                      int16_t elementary = 0) {
        node_ids_ = {id1, id2};
        physical_ = physical;
        elementary_ = elementary;
    }

    bool operator == (const BaseEdge& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) && (a.node_ids_[1] == node_ids_[1])) {
            return true;
        }

        if ((a.node_ids_[0] == node_ids_[1]) && (a.node_ids_[1] == node_ids_[0])) {
            return true;
        }

        return false;
    }

    bool operator != (const BaseEdge& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) && (a.node_ids_[1] == node_ids_[1])) {
            return false;
        }

        if ((a.node_ids_[0] == node_ids_[1]) && (a.node_ids_[1] == node_ids_[0])) {
            return false;
        }

        return true;
    }
};

struct BaseTriangle
{
public:
    std::array<uint32_t, 3> node_ids_;
    int16_t physical_;
    int16_t elementary_;

    explicit BaseTriangle(uint32_t id1 = 0.,
                          uint32_t id2 = 0.,
                          uint32_t id3 = 0.,
                          int16_t physical = 0,
                          int16_t elementary = 0) {
        node_ids_ = {id1, id2, id3};
        physical_ = physical;
        elementary_ = elementary;
    }

    bool operator == (const BaseTriangle& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) || (a.node_ids_[0] == node_ids_[1]) || (a.node_ids_[0] == node_ids_[2]))
            if ((a.node_ids_[1] == node_ids_[0]) || (a.node_ids_[1] == node_ids_[1]) || (a.node_ids_[1] == node_ids_[2]))
                if ((a.node_ids_[2] == node_ids_[0]) || (a.node_ids_[2] == node_ids_[1]) || (a.node_ids_[2] == node_ids_[2]))
                    return true;

        return false;
    }

    bool operator != (const BaseTriangle& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) || (a.node_ids_[0] == node_ids_[1]) || (a.node_ids_[0] == node_ids_[2]))
            if ((a.node_ids_[1] == node_ids_[0]) || (a.node_ids_[1] == node_ids_[1]) || (a.node_ids_[1] == node_ids_[2]))
                if ((a.node_ids_[2] == node_ids_[0]) || (a.node_ids_[2] == node_ids_[1]) || (a.node_ids_[2] == node_ids_[2]))
                    return false;

        return true;
    }
};

struct BaseTetrahedron
{
public:
    std::array<uint32_t, 4> node_ids_;
    int16_t physical_;
    int16_t elementary_;

    explicit BaseTetrahedron(uint32_t id1 = 0.,
                             uint32_t id2 = 0.,
                             uint32_t id3 = 0.,
                             uint32_t id4 = 0.,
                             int16_t elementary = 0,
                             int16_t physical = 0) {
        node_ids_ = {id1, id2, id3, id4};
        physical_ = physical;
        elementary_ = elementary;
    }

    bool operator == (const BaseTetrahedron& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) || (a.node_ids_[0] == node_ids_[1]) || (a.node_ids_[0] == node_ids_[2]) || (a.node_ids_[0] == node_ids_[3]))
            if ((a.node_ids_[1] == node_ids_[0]) || (a.node_ids_[1] == node_ids_[1]) || (a.node_ids_[1] == node_ids_[2]) || (a.node_ids_[1] == node_ids_[3]))
                if ((a.node_ids_[2] == node_ids_[0]) || (a.node_ids_[2] == node_ids_[1]) || (a.node_ids_[2] == node_ids_[2]) || (a.node_ids_[2] == node_ids_[3]))
                    if ((a.node_ids_[3] == node_ids_[0]) || (a.node_ids_[3] == node_ids_[1]) || (a.node_ids_[3] == node_ids_[2]) || (a.node_ids_[3] == node_ids_[3]))
                        return true;

        return false;
    }

    bool operator != (const BaseTetrahedron& a) const {
        if ((a.node_ids_[0] == node_ids_[0]) || (a.node_ids_[0] == node_ids_[1]) || (a.node_ids_[0] == node_ids_[2]) || (a.node_ids_[0] == node_ids_[3]))
            if ((a.node_ids_[1] == node_ids_[0]) || (a.node_ids_[1] == node_ids_[1]) || (a.node_ids_[1] == node_ids_[2]) || (a.node_ids_[1] == node_ids_[3]))
                if ((a.node_ids_[2] == node_ids_[0]) || (a.node_ids_[2] == node_ids_[1]) || (a.node_ids_[2] == node_ids_[2]) || (a.node_ids_[2] == node_ids_[3]))
                    if ((a.node_ids_[3] == node_ids_[0]) || (a.node_ids_[3] == node_ids_[1]) || (a.node_ids_[3] == node_ids_[2]) || (a.node_ids_[3] == node_ids_[3]))
                        return false;

        return true;
    }
};

typedef std::pair<uint32_t /*Node ID*/, BaseNode> Node;
typedef std::pair<uint32_t /*Edge ID*/, BaseEdge> Edge;
typedef std::pair<uint32_t /*Triangle ID*/, BaseTriangle> Triangle;
typedef std::pair<uint32_t /*Tetrahedra ID*/, BaseTetrahedron> Tetrahedron;

typedef std::unordered_map<uint32_t /*Node ID*/, BaseNode> Nodes;
typedef std::unordered_map<uint32_t /*Edge ID*/, BaseEdge> Edges;
typedef std::unordered_map<uint32_t /*Triangle ID*/, BaseTriangle> Triangles;
typedef std::unordered_map<uint32_t /*Tetrahedra ID*/, BaseTetrahedron> Tetrahedrons;
typedef std::unordered_map<uint32_t /*Tetrahedra ID*/, std::array<uint32_t, 4> /*Neighbor ID*/> TetraNeighbors;
typedef std::unordered_map<uint32_t /*Triangle ID*/, std::array<uint32_t, 3> /*Neighbor ID*/> TriangleNeighbors;

//
//using std::vector;
//struct TNode {
//    double x;
//    double y;
//    double z;
//    size_t index = 0;
//    bool exists;

//    TNode(double first, double second, double third) {
//        x = first;
//        y = second;
//        z = third;
//        exists = true;
//    }

//    bool operator <(TNode other) {
//        return index < other.index;
//    }
//};

//class Graph {
//public:
//    TNode get_neighbour(int vertex, int index);
//private:
//    vector<TNode> vertexes_;
//    vector<std::unordered_map<int, TNode>> edges_;
//};

//TNode Graph::get_neighbour(int vertex, int index) {
//    return edges_[vertex].find(index)->second;
//}

struct DelaunayCore3D
{
public:
    std::set<uint32_t> tetrahedrons_to_delete_;
    std::vector<std::tuple<BaseTriangle, uint32_t /*Outer tetrahedron ID*/, uint32_t /*Vertex number in outer tetrahedron*/>> core_front_;

    void clear();
};

std::ostream& operator<<(std::ostream& stream, BaseNode& base_node);
std::ostream& operator<<(std::ostream& stream, BaseEdge& base_edge);
std::ostream& operator<<(std::ostream& stream, BaseTriangle& base_triangle);
std::ostream& operator<<(std::ostream& stream, BaseTetrahedron& base_tetrahedron);

#endif // GRIDDATATYPES_H

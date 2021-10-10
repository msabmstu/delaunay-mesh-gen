#ifndef MATHUTILITIES_H
#define MATHUTILITIES_H

#include <griddatatypes.h>
#include <algorithm>
#include <math.h>
#include <limits>
#include <random>
#include <iterator>
#include <functional>

//------------------------------------------------------------------------------
//DISCLAMER!
//Functions below were only tested in 3D and might not support other dimentions!
//------------------------------------------------------------------------------

///Randomly selects an item from a container using iterators
template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(gen));
    return start;
}

///Check if two DOUBLE values are equal with some degree of accuracy
bool almost_equal(double a, double b, double epsilon = std::numeric_limits<double>::epsilon());

///Find the distance between two nodes
double node_distance(const BaseNode& a, const BaseNode& b);

double find_determinant(std::vector<std::vector<double>>& matrix);

///Find signed volume of a tetrahedron
double find_signed_volume(const BaseNode& a, const BaseNode& b,
                          const BaseNode& c, const BaseNode& d);

bool nodes_are_colinear(const BaseNode& a, const BaseNode& b, const BaseNode& c);

bool nodes_are_coplanar(const BaseNode& a, const BaseNode& b,
                        const BaseNode& c, const BaseNode& d);

///Find MIN and MAX values of x, y and z coordinates of the mesh
std::vector<double> find_mesh_extrema(const Nodes& nodes);

BaseNode find_triangle_center(const BaseNode& a, const BaseNode& b, const BaseNode& c);

BaseNode find_circumscribed_sphere_center(const BaseNode& a, const BaseNode& b,
                                          const BaseNode& c, const BaseNode& d);

bool node_belongs_to_sphere(const BaseNode& node,
                            const BaseNode& sphere_center, const double radius);

///Returns (-1, -1) if triangles are not adjacent and returns THE NUMBERS (0, 1 or 2) of the non-paired vertex in a form of (A, B)
std::pair<int16_t, int16_t> check_triangle_adjacency(const BaseTriangle& a, const BaseTriangle& b);

double tetrahedron_quality(const BaseNode& a, const BaseNode& b, const BaseNode& c, const BaseNode& d, std::function<double(const BaseNode&)> rho = [](const BaseNode&){return 1;});

BaseNode calculate_normal(const BaseNode& a, const BaseNode& b, const BaseNode& c);

//Check yellow george
//3.5.4.5
#endif // MATHUTILITIES_H

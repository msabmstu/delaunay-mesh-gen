#include <limits>
#include <mathutilities.h>
#include <exceptions.h>

bool almost_equal(double a, double b, double epsilon) {
    return std::abs(b - a) < epsilon;
}

double node_distance(const BaseNode &a, const BaseNode &b) {
    double res = 0;
    for (int16_t i = 0; i < 3; ++i) {
        res += std::pow((b.coords_[i] - a.coords_[i]), 2);
    }
    return std::sqrt(res);
}

double find_determinant(std::vector<std::vector<double>>& matrix) {
    double det = 0;
    std::vector<std::vector<double>> minor(matrix.size() - 1, std::vector<double>(matrix.size() - 1, 0));

    if ((matrix.size() < 1) || (matrix[0].size() != matrix.size())) {
        throw ComputationalException(201, __LINE__, __FILE__, __FUNCTION__, "Matrix, passed to calculate the determinant, is either empty or has unequal number of rows and colomns.");
    } else if (matrix.size() == 1) {
        det = matrix[0][0];
    } else if (matrix.size() == 2) {
        det = matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
    } else {
        det = 0;
        //Creating a minor to be sent in this function as a recursive input
        for (uint32_t j1 = 0; j1 < matrix.size(); ++j1) {
            for (uint32_t i = 1; i < matrix.size(); ++i) {
                uint32_t j2 = 0;
                for (uint32_t j = 0; j < matrix.size(); ++j) {
                    if (j == j1)
                        continue;
                    minor[i - 1][j2] = matrix[i][j];
                    ++j2;
                }
            }
            det += std::pow(-1.0, 1.0 + j1 + 1.0) * matrix[0][j1] * find_determinant(minor);
        }
    }
    return det;
}

double find_signed_volume(const BaseNode& a, const BaseNode& b,
                          const BaseNode& c, const BaseNode& d) {
    //Signed volume can be calculated as a determinant
    //
    // |  1    a.x   a.y   a.z |
    // |  1    b.x   b.y   b.z |
    // |  1    c.x   c.y   c.z |
    // |  1    d.x   d.y   d.z |
    std::vector<std::vector<double>> result = {{1, a.coords_[0], a.coords_[1], a.coords_[2]},
                                               {1, b.coords_[0], b.coords_[1], b.coords_[2]},
                                               {1, c.coords_[0], c.coords_[1], c.coords_[2]},
                                               {1, d.coords_[0], d.coords_[1], d.coords_[2]}};
    return find_determinant(result);
}

//This function realization works faster and uses less space than
//the realization using signed area of a triangle
bool nodes_are_colinear(const BaseNode& a, const BaseNode& b, const BaseNode& c) {

    //              (c.x - a.x)     (c.y - a.y)     (c.z - a.z)
    //Checking if  ------------- = ------------- = ------------- which is an eqution of a line
    //              (b.x - a.x)     (b.y - a.y)     (b.z - a.z)
    double fraction1 = (c.coords_[0] - a.coords_[0]) / (b.coords_[0] - a.coords_[0]);
    double fraction2 = (c.coords_[1] - a.coords_[1]) / (b.coords_[1] - a.coords_[1]);
    double fraction3 = (c.coords_[2] - a.coords_[2]) / (b.coords_[2] - a.coords_[2]);

    return almost_equal(fraction1, fraction2) && almost_equal(fraction1, fraction3);
}

//This function realization works faster and uses less space than
//the realization using signed volume
bool nodes_are_coplanar(const BaseNode& a, const BaseNode& b,
                        const BaseNode& c, const BaseNode& d) {
    //Creating 4 coefficients for an equation of a plane
    // A * x + B * y + C * z + D = 0;
    double a_coeff, b_coeff, c_coeff, d_coeff;
    //A = (b.y - a.y) * (c.z - a.z) - (c.y - a.y) * (b.z - a.z)
    a_coeff = ((b.coords_[1] - a.coords_[1]) * (c.coords_[2] - a.coords_[2]) - (c.coords_[1] - a.coords_[1]) * (b.coords_[2] - a.coords_[2]));
    //B = (c.x - a.x) * (b.z - a.z) - (b.x - a.x) * (c.z - a.z)
    b_coeff = ((c.coords_[0] - a.coords_[0]) * (b.coords_[2] - a.coords_[2]) - (b.coords_[0] - a.coords_[0]) * (c.coords_[2] - a.coords_[2]));
    //C = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    c_coeff = ((b.coords_[0] - a.coords_[0]) * (c.coords_[1] - a.coords_[1]) - (b.coords_[1] - a.coords_[1]) * (c.coords_[0] - a.coords_[0]));
    //D = -(A * a.x + B * a.y + C * a.z)
    d_coeff = -1. * (a_coeff * a.coords_[0] + b_coeff * a.coords_[1] + c_coeff * a.coords_[2]);

    //Checking if the forth point satisfies the equation of the plane
    return almost_equal((a_coeff * d.coords_[0] + b_coeff * d.coords_[1] + c_coeff * d.coords_[2] + d_coeff), 0);
}

std::vector<double> find_mesh_extrema(const Nodes& nodes) {
    //Extrema of the mesh edges extrema format: {min_x, min_y, min_z,
    //                                           max_x, max_y, max_z}
    std::vector<double> extrema;
    extrema = {nodes.find(1)->second.coords_[0],
               nodes.find(1)->second.coords_[1],
               nodes.find(1)->second.coords_[2],
               nodes.find(1)->second.coords_[0],
               nodes.find(1)->second.coords_[1],
               nodes.find(1)->second.coords_[2]};

    //Finding the max and min values for x, y, z
    for (const auto& n : nodes) {
        if (n.second.coords_[0] < extrema[0])
            extrema[0] = n.second.coords_[0];
        else if(n.second.coords_[0] > extrema[3])
            extrema[3] = n.second.coords_[0];

        if (n.second.coords_[1] < extrema[1])
            extrema[1] = n.second.coords_[1];
        else if (n.second.coords_[1] > extrema[4])
            extrema[4] = n.second.coords_[1];

        if (n.second.coords_[2] < extrema[2])
            extrema[2] = n.second.coords_[2];
        else if (n.second.coords_[2] > extrema[5])
            extrema[5] = n.second.coords_[2];
    }

    //Calculating diviation to getenerate convex hull far enough away from the initial surface mesh
    //For now I am using d = (max-min)/4, but it's a arbitrary number really, and this part can be improved later
    double d_x, d_y, d_z;
    d_x = (extrema[3] - extrema[0])/4;
    d_y = (extrema[4] - extrema[1])/4;
    d_z = (extrema[5] - extrema[2])/4;

    //Stepping away from the min and max values by d
    extrema[0] -= d_x;
    extrema[1] -= d_y;
    extrema[2] -= d_z;
    extrema[3] += d_x;
    extrema[4] += d_y;
    extrema[5] += d_z;

    return extrema;
}

BaseNode find_circumscribed_sphere_center(const BaseNode& a, const BaseNode& b,
                                          const BaseNode& c, const BaseNode& d) {
    //Solving the determinant
    //
    // | (x^2   +  y^2  +   z^2)     x     y     z     1 |
    // | (a.x^2 + a.y^2 + a.z^2)    a.x   a.y   a.z    1 |
    // | (b.x^2 + b.y^2 + b.z^2)    b.x   b.y   b.z    1 |  =  0
    // | (c.x^2 + c.y^2 + c.z^2)    c.x   c.y   c.z    1 |
    // | (d.x^2 + d.y^2 + d.z^2)    d.x   d.y   d.z    1 |
    //
    //In conjunction with
    //The general equation of a sphere with radius r centered at (x0,y0,z0) is
    //(x - center.x)^2 + (y - center.y)^2 + (z - center.z)^2 = radius^2
    double m11, m12, m13, m14;

    if (nodes_are_coplanar(a, b, c, d))
        throw ComputationalException(202, __LINE__, __FILE__, __FUNCTION__, "Points given to the find sphere center function are coplanar.");

    //This array is jut used to automate calculation of minors
    std::array<BaseNode, 4> nodes = {a, b, c, d};

    std::vector<std::vector<double>> minor(4, std::vector<double>(4, 0));
    //Calculating first minor M11
    for (uint32_t i = 0; i < 4; ++i) {
        minor[i][0] = nodes[i].coords_[0];     // | a.x     a.y     a.z     1 |
        minor[i][1] = nodes[i].coords_[1];     // | b.x     b.y     b.z     1 |
        minor[i][2] = nodes[i].coords_[2];     // | c.x     c.y     c.z     1 |
        minor[i][3] = 1;                       // | d.x     d.y     d.z     1 |
    }
    m11 = find_determinant(minor);

    //Calculating first minor M12
    for (uint32_t i = 0; i < 4; ++i) {
        minor[i][0] = std::pow(nodes[i].coords_[0], 2) +   // | (a.x^2 + a.y^2 + a.z^2)     a.y     a.z     1 |
                      std::pow(nodes[i].coords_[1], 2) +   // | (b.x^2 + b.y^2 + b.z^2)     b.y     b.z     1 |
                      std::pow(nodes[i].coords_[2], 2);    // | (c.x^2 + c.y^2 + c.z^2)     c.y     c.z     1 |
        minor[i][1] = nodes[i].coords_[1];                 // | (d.x^2 + d.y^2 + d.z^2)     d.y     d.z     1 |
        minor[i][2] = nodes[i].coords_[2];
        minor[i][3] = 1;
    }
    m12 = find_determinant(minor);

    for (uint32_t i = 0; i < 4; ++i) {
        minor[i][0] = nodes[i].coords_[0];                 // | (a.x^2 + a.y^2 + a.z^2)     a.x     a.z     1 |
        minor[i][1] = std::pow(nodes[i].coords_[0], 2) +   // | (b.x^2 + b.y^2 + b.z^2)     b.x     b.z     1 |
                      std::pow(nodes[i].coords_[1], 2) +   // | (c.x^2 + c.y^2 + c.z^2)     c.x     c.z     1 |
                      std::pow(nodes[i].coords_[2], 2);    // | (d.x^2 + d.y^2 + d.z^2)     d.x     d.z     1 |
        minor[i][2] = nodes[i].coords_[2];
        minor[i][3] = 1;
    }
    m13 = find_determinant(minor);

    for (uint32_t i = 0; i < 4; ++i) {
        minor[i][0] = nodes[i].coords_[0];                 // | (a.x^2 + a.y^2 + a.z^2)     a.x     a.y     1 |
        minor[i][1] = nodes[i].coords_[1];                 // | (b.x^2 + b.y^2 + b.z^2)     b.x     b.y     1 |
        minor[i][2] = std::pow(nodes[i].coords_[0], 2) +   // | (c.x^2 + c.y^2 + c.z^2)     c.x     c.y     1 |
                      std::pow(nodes[i].coords_[1], 2) +   // | (d.x^2 + d.y^2 + d.z^2)     d.x     d.y     1 |
                      std::pow(nodes[i].coords_[2], 2);
        minor[i][3] = 1;
    }
    m14 = find_determinant(minor);

    return BaseNode({0.5 * m12 / m11, 0.5 * m13 / m11, 0.5 * m14 / m11});
}

//TD Change epsilon() to an actual epsilon, dictated by the mesh
bool node_belongs_to_sphere(const BaseNode& node, const BaseNode& sphere_center, const double radius) {
    return node_distance(node, sphere_center) <= (radius + std::numeric_limits<double>::epsilon());
}

std::pair<int16_t, int16_t> check_triangle_adjacency(const BaseTriangle& a, const BaseTriangle& b) {
    std::set<int16_t> first = {0, 1, 2};
    std::set<int16_t> second = {0, 1, 2};

    for (uint32_t i = 0; i < 3; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            if (a.node_ids_[i] == b.node_ids_[j]) {
                first.erase(static_cast<int16_t>(i));
                second.erase(static_cast<int16_t>(j));
            }
        }
    }

    if ((first.size() == 1) && (second.size() == 1))
        return {*first.begin(), *second.begin()};

    return {-1, -1};
}

//Don't touch this. We've tried, and this 100% works
double tetrahedron_quality(const BaseNode& p, const BaseNode& a, const BaseNode& b, const BaseNode& c, std::function<double(const BaseNode&)> rho) {
    double gamma = 0;
    double sigma_a = std::min(node_distance(p, a) / (std::sqrt(rho(p) * rho(a))),
                             (std::sqrt(rho(p) * rho(a))) / node_distance(p, a));
    double sigma_b = std::min(node_distance(p, b) / (std::sqrt(rho(p) * rho(b))),
                             (std::sqrt(rho(p) * rho(b))) / node_distance(p, b));
    double sigma_c = std::min(node_distance(p, c) / (std::sqrt(rho(p) * rho(c))),
                             (std::sqrt(rho(p) * rho(c))) / node_distance(p, c));

     gamma = (12. * std::sqrt(3.) * std::abs(find_signed_volume(p, a, b, c))) /
            (std::pow((std::pow((node_distance(p, a)), 2.) + std::pow((node_distance(p, b)), 2.) + std::pow((node_distance(p, c)), 2.) +
                       std::pow((node_distance(a, b)), 2.) + std::pow((node_distance(b, c)), 2.) + std::pow((node_distance(a, c)), 2.)), 3. / 2.));

    return gamma * sigma_a * sigma_b * sigma_c;
}

//A bit of a butleg version, it returns coefficients of a normal
BaseNode calculate_normal(const BaseNode& a, const BaseNode& b, const BaseNode& c) {
  BaseNode normal;
  double buff;
  BaseNode v1({a.coords_[0] - b.coords_[0], a.coords_[1] - b.coords_[1], a.coords_[2] - b.coords_[2]});
  BaseNode v2({b.coords_[0] - c.coords_[0], b.coords_[1] - c.coords_[1], b.coords_[2] - c.coords_[2]});

  buff = std::sqrt(std::pow((v1.coords_[1] * v2.coords_[2] - v1.coords_[2] * v2.coords_[1]), 2) +
                   std::pow((v1.coords_[2] * v2.coords_[0] - v1.coords_[0] * v2.coords_[2]), 2) +
                   std::pow((v1.coords_[0] * v2.coords_[1] - v1.coords_[1] * v2.coords_[0]), 2));
  normal.coords_[0] = (v1.coords_[1] * v2.coords_[2] - v1.coords_[2] * v2.coords_[1]) / buff;
  normal.coords_[1] = (v1.coords_[2] * v2.coords_[0] - v1.coords_[0] * v2.coords_[2]) / buff;
  normal.coords_[2] = (v1.coords_[0] * v2.coords_[1] - v1.coords_[1] * v2.coords_[0]) / buff;

  return  normal;
}

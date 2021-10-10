#include <meshloader3d.h>
#include <meshunloader3d.h>
#include <mathutilities.h>
#include <delaunayadf3d.h>
#include <exceptions.h>
#include <time.h>

int main()
{
    DelaunayADF3D mygen;
    MeshLoader3D loader;
    MeshUnloader3D unloader;
    std::string file_location;
    std::shared_ptr<UnstructuredGrid> grid_ptr(new UnstructuredGrid);

    time_t start, end;

    std::cout << "Please enter the full path to the file" << std::endl;
    //std::cin >> file_location;

    loader.load("../MyDiploma/mesh_files/cube_d5_processed.msh", *grid_ptr);
/*
    mygen.initialize(grid_ptr, true);
    mygen.problem_generator();

    unloader.unload("../MyDiploma/mesh_files/boundary_cond_for_FME.msh", *grid_ptr);
    */

    time(&start);
    std::function<double(const BaseNode&)> lambda = [](const BaseNode&) { return 1. / 5.8; };

    mygen.initialize(grid_ptr, true, lambda);
    mygen.generate();
    mygen.quality_checker();

    time(&end);

    unloader.unload("../MyDiploma/mesh_files/core_tests_9.msh", *grid_ptr);


    // Calculating total time taken by the program.
    double time_taken = double(end - start);
    std::cout << "Time taken by program is : " << std::fixed
              << time_taken << std::setprecision(5) << " sec" << std::endl;

/*
    BaseNode a({0, 0, 0});
    BaseNode b({1, 0, 0});
    BaseNode c({0.5, std::sqrt(3.) / 2., 0});
    BaseNode p1({0.5, 1. / std::sqrt(12.), std::sqrt(2. / 3.)});
    BaseNode res = calculate_normal(a, c, p1);

    std::cout << res << std::endl;
*/
/*

    std::cout << nodes_are_coplanar(a, b, c, p1) << std::endl;

    std::function<double(const BaseNode&)> lambda = [](const BaseNode& node) { return 1; };

    std::cout << tetrahedron_quality(p1, a, b, c, lambda) << std::endl;
*/
/*
    BaseNode a({1, 0, 0});
    BaseNode b({0, 1, 0});
    BaseNode c({0, 0, 1});
    BaseNode p1({0, 0, 0});
    BaseNode p2({0.1, 0.1, 0.1});
    BaseNode p3({-1, 0, 0});
    BaseNode p4({0, -1, 0});
    BaseNode p5({0, 0, -1});
    BaseNode p6({1, 1, 0});
    BaseNode p7({1, 0, 1});
    BaseNode p8({0, 1, 1});


    std::function<double(const BaseNode&)> lambda = [](const BaseNode& node) { return 1; };

    std::cout << tetrahedron_quality(p1, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p2, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p3, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p4, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p5, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p6, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p7, a, b, c, lambda) << std::endl;
    std::cout << tetrahedron_quality(p8, a, b, c, lambda) << std::endl;
*/

    /* std::cout << "Please enter the full path to the output file" << std::endl;
     * std::cin >> file_location;
     * unloader.unload(file_location, grid);
     */
/*
    BaseNode a({1, 1, 1});
    BaseNode b({0, 0, 0});
    BaseNode c({2.5, 2.5, 2.5});
    BaseNode d({0, 0, 1});
    BaseNode e({0, 1, 0});

    std::cout << nodes_are_colinear(a, b, c) << std::endl;
    std::cout << nodes_are_colinear(a, b, d) << std::endl;
    std::cout << nodes_are_coplanar(a, b, c, d) << std::endl;
    std::cout << nodes_are_coplanar(a, b, d, e) << std::endl;
    std::cout << std::numeric_limits<double>::epsilon() << std::endl;
*//*
    BaseNode sphere = find_circumscribed_sphere_center(BaseNode({0, 0, 0}),
                                                       BaseNode({1, 0, 0}),
                                                       BaseNode({0, 1, 0}),
                                                       BaseNode({.5, .5, .5}));
    std::cout << sphere << std::endl;

    double thing = find_signed_volume(BaseNode({0, 0, 0}),
                                      BaseNode({1, 0, 0}),
                                      BaseNode({0, 1, 0}),
                                      BaseNode({.5, .5, .5}));
    std::cout << std::endl << "Volume: " << thing << std::endl;

    double a;
    std::vector<std::vector<double>> matrix = {{1, 2, 3}, {4, 3, 1}, {5, -2, 8}};

    a = find_determinant(matrix);

    std::cout << a << std::endl;
*/
    return 0;
}

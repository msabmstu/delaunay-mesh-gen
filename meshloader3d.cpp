#include <meshloader3d.h>
#include <exceptions.h>

void MeshLoader3D::load(const std::string& path, UnstructuredGrid& grid) const {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "MeshLoader::load()";

    //Open file to read from
    std::ifstream input_mesh_file(path);
    if (!input_mesh_file)
        throw InputFileException(101, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "Unable to open file for reading. Make sure the file exists and there is no typos in the path to the file, as well as non-UTF characters.");

    //Ignore $MeshFormat line
    input_mesh_file.ignore(256, '\n');

    //Check if the format is correct
    std::string buffer_str;
    input_mesh_file >> buffer_str;
    if (buffer_str != "2.2")
        throw  InputFileException(102, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "Version of .msh format, specified in the mesh file are not supported by this loader. It only supports version 2.2");

    //Ignore the rest of the lines, until the $EndMeshFormat.
    do {
        input_mesh_file >> buffer_str;
    }
    while(buffer_str != "$Nodes");

    //Start loading nodes
    uint32_t number_of_objects;
    uint32_t object_id;

    input_mesh_file >> number_of_objects;

    //Loading nodes one-by-one from the file and adding them
    //to the unordered map contained in Unstructured grid class.
    Nodes& nodes = grid.get_nodes();
    double new_x, new_y, new_z;
    for (uint32_t i = 0; i < number_of_objects; ++i)
    {
        input_mesh_file >> object_id >> new_x >> new_y >> new_z;
        Node new_node(object_id, std::array<double, 3>({new_x, new_y, new_z}));
        nodes.insert(std::move(new_node));
    }

    //Ignore the lines before until $Elements
    while (buffer_str != "$Elements") {
        input_mesh_file >> buffer_str;
    }


    //Start loading Finite elements
    int object_type, number_of_tags, buffer_int;
    int16_t physical, elementary;
    uint32_t id1, id2, id3, id4, number_of_tetrahedrons = 0, number_of_triangles = 0, number_of_edges = 0;
    input_mesh_file >> number_of_objects;
    Edges& edges = grid.get_edges();
    Triangles& triangles = grid.get_triangles();
    Tetrahedrons& tetrahedrons = grid.get_tetrahedrons();

    for (uint32_t i = 0; i < number_of_objects; ++i) {
        input_mesh_file >> object_id >> object_type >> number_of_tags;

        //Check the tags
        if (number_of_tags < 2)
            throw InputFileException(103, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "The number of tags for the element number " + std::to_string(object_id) + " is less than 2, which is impossible.");

        input_mesh_file >> physical >> elementary;

        //Ignore additional tags (Need to double check what to do here actually
        for (int j = 2; j < number_of_tags; ++j)
            input_mesh_file >> buffer_int;

        //Check what type of finite element it is
        switch (object_type) {
            //If it's a 2 node line:
            case 1: {
                Edge new_edge;
                ++number_of_edges;
                new_edge.first = number_of_edges;
                input_mesh_file >> id1 >> id2;
                new_edge.second.node_ids_ = {id1, id2};
                new_edge.second.physical_ = physical;
                new_edge.second.elementary_ = elementary;
                edges.insert(std::move(new_edge));
            }
                break;
            //If it's a 3 node triangle:
            case 2: {
                Triangle new_triangle;
                ++number_of_triangles;
                new_triangle.first = number_of_triangles;
                input_mesh_file >> id1 >> id2 >> id3;
                new_triangle.second.node_ids_ = {id1, id2, id3};
                new_triangle.second.physical_ = physical;
                new_triangle.second.elementary_ = elementary;
                triangles.insert(std::move(new_triangle));
            }
                break;
            //If it's a 4 node tetrahedron
            //Reading the tetrahedron elements for debug purposes.
            case 4: {
                Tetrahedron new_tetrahedron;
                ++number_of_tetrahedrons;
                new_tetrahedron.first = number_of_tetrahedrons;
                input_mesh_file >> id1 >> id2 >> id3 >> id4;
                new_tetrahedron.second.node_ids_ = {id1, id2, id3, id4};
                new_tetrahedron.second.physical_ = physical;
                new_tetrahedron.second.elementary_ = elementary;
                tetrahedrons.insert(std::move(new_tetrahedron));
            }
                break;
            //If input mesh has any extra types of elements - thrwoing an exception to guarantee correct generation down the line
            default:
                throw InputFileException(104, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "Finite elements specified in the .msh file are not supported. The loader supports only Nodes, 2-node Edges, 3-node Triangles and 4-node Tetrahedrons as elements");
        }
    }

    //Check if it was the end of the file
    input_mesh_file >> buffer_str;
    if (buffer_str != "$EndElements")
        throw InputFileException(105, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "Input file doesn't end with $EndElements tag or ends unexpectedly");

    input_mesh_file.close();
    std::cout << "File " << path << " was successfully loaded in." << std::endl;
}


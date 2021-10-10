#include <meshunloader3d.h>
#include <exceptions.h>

void MeshUnloader3D::unload(const std::string& path, UnstructuredGrid& grid) const {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "MeshUnloader::unload()";

    //Open file to write in
    std::ofstream output_mesh_file(path);
    if (!output_mesh_file)
        throw InputFileException(101, __LINE__, __FILE__, path, CURRENT_FUNCTION_, "Unable to open file for reading. Make sure the file exists and there is no typos in the path to the file, as well as non-UTF characters.");

    const Nodes& nodes = grid.get_nodes();
    const Edges& edges = grid.get_edges();
    const Triangles& triangles = grid.get_triangles();
    const Tetrahedrons& tetrahedrons = grid.get_tetrahedrons();
    uint32_t amount_of_nodes, amount_of_edges, amount_of_triangles, amount_of_tetrahedrons;
    amount_of_nodes = nodes.size();
    amount_of_edges = edges.size();
    amount_of_triangles = triangles.size();
    amount_of_tetrahedrons = tetrahedrons.size();

    //Writing necessary .msh information
    output_mesh_file <<    "$MeshFormat"   << std::endl
                     <<  "2.2 "  <<  "0 "  << sizeof (double) << std::endl
                     <<  "$EndMeshFormat"  << std::endl;

    //Unloading nodes
    output_mesh_file << "$Nodes"        << std::endl
                     << amount_of_nodes << std::endl;

    for (const auto& n : nodes)
        output_mesh_file << std::setw(10) << n.first
                         << std::setw(20) << n.second.coords_[0]
                         << std::setw(20) << n.second.coords_[1]
                         << std::setw(20) << n.second.coords_[2]
                         << std::endl;

    //Finishing Node unload, starting elements unload
    output_mesh_file << "$EndNodes" << std::endl
                     << "$Elements" << std::endl
                     << amount_of_edges + amount_of_triangles + amount_of_tetrahedrons << std::endl;

    //Starting Tetrahedron unload
    for (const auto& t : tetrahedrons)
        output_mesh_file << std::setw(10) << t.first
                         << std::setw(3)  << "4"
                         << std::setw(3)  << "2"
                         << std::setw(3)  << t.second.physical_
                         << std::setw(15) << t.second.elementary_
                         << std::setw(10) << t.second.node_ids_[0]
                         << std::setw(10) << t.second.node_ids_[1]
                         << std::setw(10) << t.second.node_ids_[2]
                         << std::setw(10) << t.second.node_ids_[3]
                         << std::endl;

    //Starting Triangle unload
    for (const auto& t : triangles)
        output_mesh_file << std::setw(10) << t.first + amount_of_tetrahedrons
                         << std::setw(3)  << "2"
                         << std::setw(3)  << "2"
                         << std::setw(3)  << t.second.physical_
                         << std::setw(15) << t.second.elementary_
                         << std::setw(10) << t.second.node_ids_[0]
                         << std::setw(10) << t.second.node_ids_[1]
                         << std::setw(10) << t.second.node_ids_[2]
                         << std::endl;

    //Starting Edge unload
    for (const auto& e : edges)
        output_mesh_file << std::setw(10) << e.first + amount_of_tetrahedrons + amount_of_triangles
                         << std::setw(3)  << "1"
                         << std::setw(3)  << "2"
                         << std::setw(3)  << e.second.physical_
                         << std::setw(15) << e.second.elementary_
                         << std::setw(10) << e.second.node_ids_[0]
                         << std::setw(10) << e.second.node_ids_[1]
                         << std::endl;

    output_mesh_file << "$EndElements" << std::endl;

    output_mesh_file.close();

    std::cout << "Mesh was successfully unloaded in " << path << std::endl;
}


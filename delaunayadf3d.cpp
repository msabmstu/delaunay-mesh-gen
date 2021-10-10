#include <delaunayadf3d.h>
#include <mathutilities.h>
#include <meshunloader3d.h>

void DelaunayADF3D::generate() {
    //generating convex hull
    generate_convex_hull();


    //Generating initial triangulation by adding all of the points except the points of a convex hull
    for (const auto& n : grid_ptr_->get_nodes())
        if (n.first > 8) {
            insert_node(n);
            std::cout << "Node number " + std::to_string(n.first) + " has been inserted" << std::endl;
        }

    //Untested function
    while (!free_tetrahedron_ids_.empty()) {
        //If missing tetra is at the end of set
        if (free_tetrahedron_ids_.find(greatest_tetrahedron_id_) == free_tetrahedron_ids_.end()) {
            free_tetrahedron_ids_.erase(greatest_tetrahedron_id_);
            --greatest_tetrahedron_id_;
        }
        else {
            tetra_id_change(greatest_tetrahedron_id_, *free_tetrahedron_ids_.begin());
            --greatest_tetrahedron_id_;
            free_tetrahedron_ids_.erase(*free_tetrahedron_ids_.begin());
        }
    }
    std::cout << "Finished surface node insertion into a DT" << std::endl;

    MeshUnloader3D unloader;
    unloader.unload("../MyDiploma/mesh_files/before_boundary_recovery.msh", *grid_ptr_);

    recover_boundaries();

    unloader.unload("../MyDiploma/mesh_files/after_bounadry_recovery.msh", *grid_ptr_);

    //generate_inner_points();
    generate_delaunay_only();

    std::cout << "Mesh generation has been finished successfully" << std::endl;
}

void DelaunayADF3D::initialize(std::shared_ptr<UnstructuredGrid> grid_ptr, bool counter_clockwise, std::function<double(const BaseNode&)> rho) {
    if (counter_clockwise == true) {
        inner_area_ = 0;
        outer_area_ = 2;
    }
    else {
        inner_area_ = 2;
        outer_area_ = 0;
    }
    counter_clockwise_orientation_ = counter_clockwise;
    rho_ = rho;
    meshed_area_ = 1;
    grid_ptr_ = grid_ptr;
    greatest_node_id_ = grid_ptr_->get_nodes().size();
    greatest_edge_id_ = grid_ptr_->get_edges().size();
    greatest_triangle_id_ = grid_ptr_->get_triangles().size();
    greatest_tetrahedron_id_ = grid_ptr_->get_tetrahedrons().size();
}

void DelaunayADF3D::generate_convex_hull() {
    //Getting the mins and maxs for x, y, z coordinates of points
    //The order of those points is: min_x, min_y, min_z
    //                              max_x, max_y, max_z
    std::vector<double> extrema = find_mesh_extrema(grid_ptr_->get_nodes());

    //Creating 8 new nodes for a convex hull to cover the initial mesh
    //New nodes are numbered in the order shown below
    //
    //                    Z
    //                    ^
    //                    |
    //                4----------3
    //               /|   |     /|
    //              / |   |    / |
    //             /  |   |   /  |
    //            8---+------7   |
    //            |   |   +- |-- | -> Y
    //            |   1--/---+---2
    //            |  /  /    |  /
    //            | /  v     | /
    //            |/  X      |/
    //            5----------6
    //
    std::vector<Node> new_nodes;
    new_nodes.emplace_back(1, BaseNode({extrema[0], extrema[1], extrema[2]}));
    new_nodes.emplace_back(2, BaseNode({extrema[0], extrema[4], extrema[2]}));
    new_nodes.emplace_back(3, BaseNode({extrema[0], extrema[4], extrema[5]}));
    new_nodes.emplace_back(4, BaseNode({extrema[0], extrema[1], extrema[5]}));
    new_nodes.emplace_back(5, BaseNode({extrema[3], extrema[1], extrema[2]}));
    new_nodes.emplace_back(6, BaseNode({extrema[3], extrema[4], extrema[2]}));
    new_nodes.emplace_back(7, BaseNode({extrema[3], extrema[4], extrema[5]}));
    new_nodes.emplace_back(8, BaseNode({extrema[3], extrema[1], extrema[5]}));

    //Moving initial nodes back, so it'll be easier to keep track of Convex Hull Nodes
    //Going through the entire map and changing all node IDs to ID + 8
    //Thus creating 8 new spaces for the new Convex Hull nodes to fill
    Nodes& nodes = grid_ptr_->get_nodes();
    Edges& edges = grid_ptr_->get_edges();
    Triangles& triangles = grid_ptr_->get_triangles();
    Tetrahedrons& tetrahedrons = grid_ptr_->get_tetrahedrons();
    for (uint32_t i = nodes.size(); i > 0; --i) {
        nodes.insert({i + 8, nodes.find(i)->second});
        nodes.erase(i);
    }

    //Changing all dependant Triangles, Tetrahedrons and Edges so the initial grid won't be affected
    for (uint32_t i = edges.size(); i > 0; --i) {
        for (auto& n : edges.find(i)->second.node_ids_)
            n += 8;
    }
    for (uint32_t i = triangles.size(); i > 0; --i) {
        for (auto& n : triangles.find(i)->second.node_ids_)
            n += 8;
    }
    for (uint32_t i = tetrahedrons.size(); i > 0; --i) {
        for (auto& n : tetrahedrons.find(i)->second.node_ids_)
            n += 8;
    }

    //Adding Convex Hull Nodes to the grid
    for (const auto& n : new_nodes)
        grid_ptr_->get_nodes().insert(std::move(n));

    //Generating tetrahedrons using new points
    //Those tetrahedrons are: {2, 5, 4, 1}
    //                        {2, 7, 5, 6}
    //                        {2, 4, 7, 3}
    //                        {4, 5, 7, 8}
    //                        {2, 4, 5, 7}
    std::vector<Tetrahedron> new_tetrahedrons;
    new_tetrahedrons.emplace_back(1, BaseTetrahedron(2, 5, 4, 1));
    new_tetrahedrons.emplace_back(2, BaseTetrahedron(2, 7, 5, 6));
    new_tetrahedrons.emplace_back(3, BaseTetrahedron(2, 4, 7, 3));
    new_tetrahedrons.emplace_back(4, BaseTetrahedron(4, 5, 7, 8));
    new_tetrahedrons.emplace_back(5, BaseTetrahedron(2, 4, 5, 7));

    //Clearing tetrahedron container to make sure that elements there won't interfere
    //with algorithm. After this step there will only be 5 tetrahedrons in the mesh.
    tetrahedrons.clear();

    //Adding Convex Hull Tetrahedrons to the grid
    for (const auto& t : new_tetrahedrons)
        grid_ptr_->get_tetrahedrons().insert(std::move(t));

    //Maintaining map of tetrahedron neighbors after we added new elements to the grid
    neighbors_.insert({1, {0, 0, 0, 5}});
    neighbors_.insert({2, {0, 0, 0, 5}});
    neighbors_.insert({3, {0, 0, 0, 5}});
    neighbors_.insert({4, {0, 0, 0, 5}});
    neighbors_.insert({5, {4, 2, 3, 1}});

    //Maintaining correct IDs
    greatest_node_id_ += 8;
    greatest_tetrahedron_id_ += 5;

    std::cout << "Completed generation of a convex hull" << std::endl;
}

void DelaunayADF3D::insert_node(const Node& new_node){
    //Generating CORE which consists of the tetrahedrons that should be deleted
    //And triangle adges surrounding them
    DelaunayCore3D core;
    generate_core(core, new_node.second);
    triangulate_core(core, new_node);
}

void DelaunayADF3D::generate_core_front(DelaunayCore3D& core) const {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::assign_triangle_edges_of_the_core()";
    const std::string CURRENT_STEP_ = "CORE generation";

    //Clearing core front to make sure that nothing interferes with the generation
    core.core_front_.clear();

    Tetrahedrons& tetrahedrons = grid_ptr_->get_tetrahedrons();
    bool neighbor_is_included;
    uint32_t current_neighbor_id;
    uint32_t vertex_number;

    //Checking which of the 4 tetrahedrons should be added to the core
    for (const auto& t : core.tetrahedrons_to_delete_) {
        //Getting IDS of neighbors of every tetrahedron from the core
        for (uint32_t i = 0; i < 4; ++i) {
            neighbor_is_included = false;
            current_neighbor_id = neighbors_.find(t)->second[i];
            //Checking if neighbor even exists
            if (current_neighbor_id != 0) {
                //Checking what triangles are needed to be added by checking what neighbors are included in the core
                for (const auto& dt : core.tetrahedrons_to_delete_) {
                    if (current_neighbor_id == dt) {
                        neighbor_is_included = true;
                        break;
                    }
                }
            }
            if (!neighbor_is_included) {
                if (current_neighbor_id != 0)
                    for (uint32_t j = 0; j < 4; ++j)
                        if (neighbors_.find(current_neighbor_id)->second[j] == t) {
                            vertex_number = j;
                            break;
                        }

                //With how we defined an orientation of the tetrahedron, the correct numeration of triangles is:
                //Opposite to node 1: {2, 4, 3}
                //Opposite to node 2: {1, 3, 4}
                //Opposite to node 3: {1, 4, 2}
                //Opposite to node 4: {1, 2, 3}
                switch (i) {
                    case 0: {
                        core.core_front_.push_back({BaseTriangle(tetrahedrons.find(t)->second.node_ids_[1],
                                                                 tetrahedrons.find(t)->second.node_ids_[3],
                                                                 tetrahedrons.find(t)->second.node_ids_[2]),
                                                    current_neighbor_id, vertex_number});
                        break;
                    }
                    case 1: {
                        core.core_front_.push_back({BaseTriangle(tetrahedrons.find(t)->second.node_ids_[0],
                                                                 tetrahedrons.find(t)->second.node_ids_[2],
                                                                 tetrahedrons.find(t)->second.node_ids_[3]),
                                                    current_neighbor_id, vertex_number});
                        break;
                    }
                    case 2: {
                        core.core_front_.push_back({BaseTriangle(tetrahedrons.find(t)->second.node_ids_[0],
                                                                 tetrahedrons.find(t)->second.node_ids_[3],
                                                                 tetrahedrons.find(t)->second.node_ids_[1]),
                                                    current_neighbor_id, vertex_number});
                        break;
                    }
                    case 3: {
                        core.core_front_.push_back({BaseTriangle(tetrahedrons.find(t)->second.node_ids_[0],
                                                                 tetrahedrons.find(t)->second.node_ids_[1],
                                                                 tetrahedrons.find(t)->second.node_ids_[2]),
                                                    current_neighbor_id, vertex_number});
                        break;
                    }
                    default:
                        throw GeneratorException(302, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Fatal error. If you are seeing this error - the problem is in iterator or Neighbors container.");
                }
            }
        }
    }
}

void DelaunayADF3D::generate_core(DelaunayCore3D& core, const BaseNode& new_node) const {
    //Clearing the core so it can be built from the ground up
    core.clear();

    //Adding BASE element to the core
    uint32_t base_id = get_base_elemnt_id(new_node);
    core.tetrahedrons_to_delete_.insert(base_id);

    //Adding neighbors of BASE element to check if they need to be added to the CORE
    std::set<uint32_t> neighbors_to_check;
    for (auto n : neighbors_.find(base_id)->second)
        if (n != 0)
            neighbors_to_check.insert(n);

    uint32_t current_neighbor_id = 0;
    BaseTetrahedron current_neighbor;

    //Checking every neighbor of the CORE elements to see if they need to be added to the CORE
    while (not neighbors_to_check.empty()) {
        current_neighbor_id = *neighbors_to_check.begin();
        if (!is_delaunay(current_neighbor_id, new_node)) {
            //Adding the element that failed Delaunay test, along with the neighbors of it
            core.tetrahedrons_to_delete_.insert(current_neighbor_id);
            for (const auto &n : neighbors_.find(current_neighbor_id)->second)
                //Checking if the neighbor exists and is not marked as a tetrahedron to delete
                //to avoid loops and excessive checks
                if ((n != 0) && (core.tetrahedrons_to_delete_.find(n) == core.tetrahedrons_to_delete_.end()))
                    neighbors_to_check.insert(n);
        }
        neighbors_to_check.erase(current_neighbor_id);
    }

    //Finalizeing core generation by addition of faces
    generate_core_front(core);

    correct_core(core, new_node);
}

void DelaunayADF3D::correct_core(DelaunayCore3D& core, const BaseNode& new_node) const {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::correct_core()";
    const std::string CURRENT_STEP_ = "CORE correction";

    bool needs_to_be_corrected = false;
    //After the CORE is generated, it needs to be corrected by
    //performing a visibility check to each triangle edge.
    for (const auto& t : core.core_front_) {
        if (!is_visible(std::get<0>(t), new_node)) {
            std::pair<uint32_t, uint32_t> tetrahedrons_to_delete = find_adjacent_tetrahedra(std::get<0>(t), grid_ptr_->get_tetrahedrons());
            if ((tetrahedrons_to_delete.first > 0) && (tetrahedrons_to_delete.second > 0)) {
                core.tetrahedrons_to_delete_.erase(tetrahedrons_to_delete.first);
                core.tetrahedrons_to_delete_.erase(tetrahedrons_to_delete.second);

                needs_to_be_corrected = true;
            }
            else
                throw GeneratorException(303, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "At least one of the CORE triangles have failed the visibility check and CORE correction function failed to fix it.");
        }
        /*
        if (!is_visible(std::get<0>(t), new_node)) {
            std::pair<uint32_t, uint32_t> tetrahedrons_to_delete = find_adjacent_tetrahedra(std::get<0>(t), grid_ptr_->get_tetrahedrons());
            if ((tetrahedrons_to_delete.first > 0) && (tetrahedrons_to_delete.second > 0)) {
                core.tetrahedrons_to_delete_.insert(tetrahedrons_to_delete.first);
                core.tetrahedrons_to_delete_.insert(tetrahedrons_to_delete.second);

                needs_to_be_corrected = true;
            }
            else
                throw GeneratorException(303, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "At least one of the CORE triangles have failed the visibility check and CORE correction function failed to fix it.");
        }*/

    }

    if (needs_to_be_corrected) {
        generate_core_front(core);
        correct_core(core, new_node);
    }
}

void DelaunayADF3D::triangulate_core(const DelaunayCore3D &core, const Node& new_node, bool zonal_division_active) {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::triangulate_core()";
    const std::string CURRENT_STEP_ = "CORE triangulation";

    Tetrahedrons& tetrahedrons = grid_ptr_->get_tetrahedrons();
    Tetrahedrons new_tetrahedrons;
    uint32_t new_tetrahedron_id = 0;

    //Deleting tetrahedrons included in the CORE
    for (auto t : core.tetrahedrons_to_delete_) {
        tetrahedrons.erase(t);
        free_tetrahedron_ids_.insert(t);

        if (zonal_division_active)
            zonal_division_.erase(t);

    }

    //Inserting new tertrahedrons
    for (const auto& t : core.core_front_) {
        //Getting a correct ID for a new element
        if (!free_tetrahedron_ids_.empty()) {
            new_tetrahedron_id = *free_tetrahedron_ids_.begin();
            free_tetrahedron_ids_.erase(free_tetrahedron_ids_.begin());
        }
        else {
            ++greatest_tetrahedron_id_;
            new_tetrahedron_id = greatest_tetrahedron_id_;
        }

        //Clearing old and adding new neighbor elements in the storage
        neighbors_.erase(new_tetrahedron_id);
        neighbors_.insert({new_tetrahedron_id, std::array<uint32_t, 4>({0, 0, 0, 0})});

        //Creating and inserting a new tetrahedron
        Tetrahedron new_tetrahedron(new_tetrahedron_id, BaseTetrahedron(std::get<0>(t).node_ids_[0], std::get<0>(t).node_ids_[1], std::get<0>(t).node_ids_[2], new_node.first));
        new_tetrahedrons.insert(std::move(new_tetrahedron));

        if (zonal_division_active)
            zonal_division_.insert({new_tetrahedron_id, inner_area_});


        //Fixing neighboring relationships between new elements and elements outside of the CORE
        if (std::get<1>(t) != 0)
            neighbors_.find(std::get<1>(t))->second[std::get<2>(t)] = new_tetrahedron_id;

        //Fixing neighboring relationships between elements outside of the CORE and new elements
        neighbors_.find(new_tetrahedron_id)->second[3] = std::get<1>(t);
    }

    //Fixing inner CORE adjacency relationships
    std::queue<BaseTriangle> triangle_queue;
    BaseTriangle current_triangle;
    BaseTriangle triangle_to_check;
    uint32_t current_tetrahedron_id;
    uint32_t current_neighbor_id;
    int16_t repetition;
    std::pair<int16_t, int16_t> adjacency_result;
    for (const auto& t : core.core_front_){
        triangle_queue.push(std::get<0>(t));
    }

    while (!triangle_queue.empty()) {
        current_triangle = triangle_queue.front();
        repetition = 0;
        for (const auto& t : core.core_front_) {
            triangle_to_check = std::get<0>(t);
            adjacency_result = check_triangle_adjacency(current_triangle, triangle_to_check);
            if ((adjacency_result.first != -1) && (adjacency_result.second != -1)) {
                //find_adjacent_tetrahedra.second should be always 0 in this case
                current_tetrahedron_id = find_adjacent_tetrahedra(current_triangle, new_tetrahedrons).first;
                current_neighbor_id = find_adjacent_tetrahedra(triangle_to_check, new_tetrahedrons).first;

                if ((current_neighbor_id == 0) || (current_tetrahedron_id == 0))
                    throw GeneratorException(304, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Tetrahedron ID, found by checking an original edge of the core, is equal to 0. This error should not be thrown, if the ID numeration system is working as intended.");

                neighbors_.find(current_tetrahedron_id)->second[adjacency_result.first] = current_neighbor_id;
                neighbors_.find(current_neighbor_id)->second[adjacency_result.second] = current_tetrahedron_id;
                ++repetition;
            }
            if(repetition == 3)
                break;
        }
        triangle_queue.pop();
    }

    //Adding all of the new tetrahedra to the grid
    for (auto& t : new_tetrahedrons) {
        tetrahedrons.insert(std::move(t));
    }
}

std::pair<uint32_t, uint32_t> DelaunayADF3D::find_adjacent_tetrahedra(const BaseTriangle& triangle, const Tetrahedrons& tetrahedrons) const {
    //TD do better
    int16_t repetition = 0;
    bool found = false;
    std::pair<uint32_t, uint32_t> result = {0, 0};

    for (const auto& t : tetrahedrons) {
        repetition = 0;
        for (const auto& tn : t.second.node_ids_) {
            for (const auto& trn : triangle.node_ids_) {
                if (tn == trn) {
                    ++repetition;
                    break;
                }
            }
            if (repetition == 3) {
                if (!found) {
                    result.first = t.first;
                    found = true;
                }
                else {
                    result.second = t.first;
                }
            }
        }
    }
    return result;
}

uint32_t DelaunayADF3D::get_base_elemnt_id(const BaseNode& node_to_insert) const {
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::get_base_elemnt_id()";
    const std::string CURRENT_STEP_ = "CORE generation";

    uint32_t id = 0;

    //This can be replaced with a more 'Smart' way to get a first starting tetrahedron
    //But for now the pick is pretty arbitrary. Currently I just pick tetrahedron with the ID = 1
    id = 1;

    //l_coord - are L-coordinates of a new node in relation to the BASE candidate tetrahedron's faces
    std::array<double, 4> l_coord;
    BaseTetrahedron base_candidate;

    //base_nodes - verticies of a current base_candidate tetrahedron
    std::array<BaseNode, 4> base_nodes;
    const Nodes& nodes = grid_ptr_->get_nodes();

    bool base_not_found = true;
    uint32_t test_for_inf_loop = grid_ptr_->get_tetrahedrons().size();
    std::vector<uint32_t> negative_coordinates;

    while (base_not_found) {
        //Checking for the infinite loop.
        --test_for_inf_loop;

        //TD change the test_for_inf_loop to a lower value, right now it's kinda pointless to have it this long
        //Going through all tetrahedrons if case the L coordinates method got stuck in inf loop
        if(test_for_inf_loop == 0) {
            for (const auto& t : grid_ptr_->get_tetrahedrons()) {
                for (uint32_t i = 0; i < 4; ++i)
                    base_nodes[i] = nodes.find(t.second.node_ids_[i])->second;

                //Calculating all L-coordinates for the current point in the base tetrahedron
                //They are not divided by Volume of a tetrahedron to check, because only the sign of the L coordinate is of any importance
                l_coord[0] = find_signed_volume(node_to_insert, base_nodes[1], base_nodes[2], base_nodes[3]);
                l_coord[1] = find_signed_volume(base_nodes[0], node_to_insert, base_nodes[2], base_nodes[3]);
                l_coord[2] = find_signed_volume(base_nodes[0], base_nodes[1], node_to_insert, base_nodes[3]);
                l_coord[3] = find_signed_volume(base_nodes[0], base_nodes[1], base_nodes[2], node_to_insert);

                for (uint32_t i = 0; i < 4; ++i) {
                    //If L-coordinates are < 0, then moving on with the next tetrahedron
                    if (l_coord[i] < 0 - 10 * std::numeric_limits<double>::epsilon()) {
                        if (neighbors_.find(id)->second[i] == 0)
                            throw GeneratorException(301, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Most likely point, which was called to be added, is outside of the convex hull.");

                        break;
                    }

                    //If all coordinates are greater than 0 range - then the point lies inside of this candidate,
                    //thus base was found. Stopping the search
                    if (i == 3) {
                        return t.first;
                    }
                }
            }

            MeshUnloader3D unloader;
            unloader.unload("../MyDiploma/mesh_files/last_crash_step", *grid_ptr_);

            throw GeneratorException(300, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Function finding the BASE tetrahedron ID failed to do so after a full detour search. \n"
                                                                                                "it might happen if the point lies on the one of the tetrahedrons face.");
        }

        //Fetching base_candidate and its nodes from the grid
        base_candidate = grid_ptr_->get_tetrahedrons().find(id)->second;
        for (uint32_t i = 0; i < 4; ++i)
            base_nodes[i] = nodes.find(base_candidate.node_ids_[i])->second;

        //Calculating all L-coordinates for the current point in the base tetrahedron
        //They are not divided by Volume of a tetrahedron to check, because only the sign of the L coordinate is of any importance
        l_coord[0] = find_signed_volume(node_to_insert, base_nodes[1], base_nodes[2], base_nodes[3]);
        l_coord[1] = find_signed_volume(base_nodes[0], node_to_insert, base_nodes[2], base_nodes[3]);
        l_coord[2] = find_signed_volume(base_nodes[0], base_nodes[1], node_to_insert, base_nodes[3]);
        l_coord[3] = find_signed_volume(base_nodes[0], base_nodes[1], base_nodes[2], node_to_insert);

        negative_coordinates.clear();
        for (uint32_t i = 0; i < 4; ++i) {
            //If L-coordinates are < 0, saving the negative L-coordinate in negative_coordinates vector
            if (l_coord[i] < 0 - std::numeric_limits<double>::epsilon()) {
                if (neighbors_.find(id)->second[i] == 0)
                    throw GeneratorException(301, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Most likely point, which was called to be added, is outside of the convex hull.");

                //Saving all negative L-coordinate ids to randomize it later
                negative_coordinates.push_back(i);
            }

            //If all coordinates are greater than 0 range - then the point lies inside of this candidate,
            //thus base was found. Stopping the WHILE loop, otherwise - picking a random vertex, which has a negative L-coordinate,
            //and assigning his opposite neighbor as a new base candidate
            if (i == 3) {
                if (negative_coordinates.empty())
                    base_not_found = false;
                else
                    id = neighbors_.find(id)->second[*select_randomly(negative_coordinates.begin(), negative_coordinates.end())];
            }
        }
    }
    return id;
}

//Untested function
void DelaunayADF3D::tetra_id_change(uint32_t swap_from, uint32_t swap_to){
    //Macro __FUNCTION__ can also be used instead of CURRENT_FUNCTION_,
    //but it gives less information on the function, in which the error occured.
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::tetra_id_swap()";
    const std::string CURRENT_STEP_ = "Correction of missing tetrahedra ids after point insertion";

    Tetrahedrons& tetrahedrons = grid_ptr_->get_tetrahedrons();
    if ((tetrahedrons.find(swap_from) == tetrahedrons.end()) || (tetrahedrons.find(swap_to) == tetrahedrons.end()))
        throw GeneratorException(305, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Failed to find provided ID's of tetrahedra.");

    for (const auto& i : neighbors_.find(swap_from)->second)
        for (auto n : neighbors_.find(i)->second)
            if (n == swap_from)
                n = swap_to;

    BaseTetrahedron sf_copy = tetrahedrons.find(swap_from)->second;
    tetrahedrons.insert({swap_to, sf_copy});
    tetrahedrons.erase(swap_from);

}

bool DelaunayADF3D::is_delaunay(uint32_t tetrahedron_id, const BaseNode& node_to_check) const {
    //This is a low-level function, so it's a bit messy
    //In the normal wiev this function checks if point belongs to sphere.
    //node_to_check - node we are checking if it's inside the sircumscribed sphere
    //find_circumscribed_sphere_center(tetrahedron vertecies) - finding the ceter of the sphere of a tetrahedron
    //node_distance(node_to_check, one of the tetrahedron vertecies) - finding the sphere radius
    //The node CENTER was created in order to save some memory
   const BaseNode center = find_circumscribed_sphere_center(grid_ptr_->get_nodes().find(grid_ptr_->get_tetrahedrons().find(tetrahedron_id)->second.node_ids_[0])->second,
                                                            grid_ptr_->get_nodes().find(grid_ptr_->get_tetrahedrons().find(tetrahedron_id)->second.node_ids_[1])->second,
                                                            grid_ptr_->get_nodes().find(grid_ptr_->get_tetrahedrons().find(tetrahedron_id)->second.node_ids_[2])->second,
                                                            grid_ptr_->get_nodes().find(grid_ptr_->get_tetrahedrons().find(tetrahedron_id)->second.node_ids_[3])->second);
    return !node_belongs_to_sphere(node_to_check, center, node_distance(center, grid_ptr_->get_nodes().find(grid_ptr_->get_tetrahedrons().find(tetrahedron_id)->second.node_ids_[0])->second));
}

bool DelaunayADF3D::is_visible(const BaseTriangle& triangle, const BaseNode& node) const {
    const Nodes& nodes = grid_ptr_->get_nodes();
    if (nodes_are_coplanar(nodes.find(triangle.node_ids_[0])->second,
                           nodes.find(triangle.node_ids_[1])->second,
                           nodes.find(triangle.node_ids_[2])->second,
                           node)) {
        return false;
    }
    return find_signed_volume(nodes.find(triangle.node_ids_[0])->second,
                              nodes.find(triangle.node_ids_[1])->second,
                              nodes.find(triangle.node_ids_[2])->second,
                              node) > 0 + 10 * std::numeric_limits<double>::epsilon();
    //TD change prescision
}

void DelaunayADF3D::recover_boundaries() {

    find_missing_edges();

    Nodes& nodes = grid_ptr_->get_nodes();
    Triangles& triangles = grid_ptr_->get_triangles();
    std::queue<Node> new_nodes;
    BaseEdge current_edge;
    BaseTriangle current_triangle;
    std::list<std::tuple<uint32_t, uint32_t, BaseEdge>> triangle_steiner_list;

    while (!missing_edges_.empty()) {
        //Pick an edge, find points to insert
        for (auto& e : missing_edges_) {
            triangle_steiner_list = generate_triangle_queue(e);       

            //Find and insert Steiner points
            for (auto& t : triangle_steiner_list) {
                current_edge = std::get<2>(t);

                //Generates a new node on the middle of the edge
                Node new_node({greatest_node_id_ + 1,
                               BaseNode({(nodes.find(current_edge.node_ids_[0])->second.coords_[0] + nodes.find(current_edge.node_ids_[1])->second.coords_[0]) / 2,
                                         (nodes.find(current_edge.node_ids_[0])->second.coords_[1] + nodes.find(current_edge.node_ids_[1])->second.coords_[1]) / 2,
                                         (nodes.find(current_edge.node_ids_[0])->second.coords_[2] + nodes.find(current_edge.node_ids_[1])->second.coords_[2]) / 2})});
                ++greatest_node_id_;

                current_triangle = triangles.find(std::get<0>(t))->second;

                for (uint32_t i = 0; i < 3; ++i) {
                    if ((current_triangle.node_ids_[i] != current_edge.node_ids_[0]) && (current_triangle.node_ids_[i] != current_edge.node_ids_[1])) {
                        Triangle new_triangle1({std::get<0>(t),
                                               BaseTriangle(current_triangle.node_ids_[i], current_triangle.node_ids_[(i + 1) % 3], new_node.first, 1)});
                        Triangle new_triangle2({greatest_triangle_id_ + 1,
                                               BaseTriangle(current_triangle.node_ids_[(i + 2) % 3], current_triangle.node_ids_[i], new_node.first, 1)});
                        ++greatest_triangle_id_;

                        triangles.erase(std::get<0>(t));
                        triangles.insert(std::move(new_triangle1));
                        triangles.insert(std::move(new_triangle2));
                    }
                }

                current_triangle = triangles.find(std::get<1>(t))->second;

                for (uint32_t i = 0; i < 3; ++i) {
                    if ((current_triangle.node_ids_[i] != current_edge.node_ids_[0]) && (current_triangle.node_ids_[i] != current_edge.node_ids_[1])) {
                        Triangle new_triangle1({std::get<1>(t),
                                               BaseTriangle(current_triangle.node_ids_[i], current_triangle.node_ids_[(i + 1) % 3], new_node.first, 1)});
                        Triangle new_triangle2({greatest_triangle_id_ + 1,
                                               BaseTriangle(current_triangle.node_ids_[(i + 2) % 3], current_triangle.node_ids_[i], new_node.first, 1)});
                        ++greatest_triangle_id_;

                        triangles.erase(std::get<1>(t));
                        triangles.insert(std::move(new_triangle1));
                        triangles.insert(std::move(new_triangle2));
                    }
                }

                new_nodes.push(std::move(new_node));
                nodes.insert(new_nodes.back());
                break;
                //This is utterly fucking retarded
                //TODO: Fix this.
            }

            //Insert steiner points in triangulation
            while (!new_nodes.empty()) {
                insert_node(new_nodes.back());
                new_nodes.pop();
            }
        }

        find_missing_edges();
    }

    std::cout << "Boundary surface was successfully reconstructed" << std::endl;
}

void DelaunayADF3D::find_missing_edges() {
    missing_connections_list_.clear();
    missing_edges_.clear();

    missing_connections_list_ = std::vector<std::set<uint32_t>>(grid_ptr_->get_nodes().size() + 1);

    for (const auto& t : grid_ptr_->get_triangles())
        for (uint32_t i = 0; i < 3; ++i)
            for (uint32_t j = 0; j < 3; ++j)
                if (i != j)
                    missing_connections_list_[t.second.node_ids_[i]].insert(t.second.node_ids_[j]);

    for (const auto& t : grid_ptr_->get_tetrahedrons())
        for (uint32_t i = 0; i < 4; ++i)
            for (uint32_t j = 0; j < 4; ++j)
                if (i != j)
                    missing_connections_list_[t.second.node_ids_[i]].erase(t.second.node_ids_[j]);

    for (uint32_t i = 1; i < missing_connections_list_.size(); ++i)
        if (!missing_connections_list_[i].empty())
            for (const auto& j : missing_connections_list_[i])
                if (i < j)
                    missing_edges_.push_back(BaseEdge(i, j));
}

BaseEdge DelaunayADF3D::find_longest_edge(const BaseTriangle& triangle) {
    BaseEdge result;
    Nodes& nodes = grid_ptr_->get_nodes();
    double max_dist = node_distance(nodes.find(triangle.node_ids_[0])->second, nodes.find(triangle.node_ids_[1])->second);
    uint16_t flag = 0;

    if (max_dist < node_distance(nodes.find(triangle.node_ids_[0])->second, nodes.find(triangle.node_ids_[2])->second)) {
        max_dist = node_distance(nodes.find(triangle.node_ids_[0])->second, nodes.find(triangle.node_ids_[2])->second);
        flag = 1;
    }

    if (max_dist < node_distance(nodes.find(triangle.node_ids_[1])->second, nodes.find(triangle.node_ids_[2])->second))
        flag = 2;


    if (flag == 0)
        result = BaseEdge(triangle.node_ids_[0], triangle.node_ids_[1]);

    if (flag == 1)
        result = BaseEdge(triangle.node_ids_[0], triangle.node_ids_[2]);

    if (flag == 2)
        result = BaseEdge(triangle.node_ids_[1], triangle.node_ids_[2]);


    return result;
}

/*
void DelaunayADF3D::build_surface_neighbors() {
    std::pair<int16_t, int16_t> opposite_nodes;

    for (const auto& t : grid_ptr_->get_triangles())
        surface_neighbors_.insert({t.first, {0, 0, 0}});

    for (const auto& t1 : grid_ptr_->get_triangles())
        for (const auto& t2 : grid_ptr_->get_triangles()) {
            if (t1.first != t2.first)
                opposite_nodes = check_triangle_adjacency(t1.second, t2.second);

            if ((opposite_nodes.first != -1) && (opposite_nodes.second != -1)) {
                surface_neighbors_.find(t1.first)->second[opposite_nodes.first] = t2.first;
                surface_neighbors_.find(t2.first)->second[opposite_nodes.second] = t1.first;
            }
        }
}
*/

std::pair<uint32_t, uint32_t> DelaunayADF3D::find_triangles_by_edge(const BaseEdge& edge) {
    uint32_t triangles_found = 0;
    uint32_t points_coinsided = 0;
    std::pair<uint32_t, uint32_t> result;

    for (const auto& t : grid_ptr_->get_triangles()) {
        points_coinsided = 0;
        for (uint32_t i = 0; i < 3; ++i) {
            if ((edge.node_ids_[0] == t.second.node_ids_[i]) || (edge.node_ids_[1] == t.second.node_ids_[i]))
                ++points_coinsided;
        }
        if (points_coinsided == 2) {
            if (triangles_found == 0) {
                ++triangles_found;
                result.first = t.first;
            }
            else if (result.first > t.first) {
                result.second = result.first;
                result.first = t.first;
                break;
            }
            else {
                result.second = t.first;
                break;
            }
        }
    }

    return result;
}

std::list<std::tuple<uint32_t, uint32_t, BaseEdge>> DelaunayADF3D::generate_triangle_queue(BaseEdge& edge) {
    bool was_updated = true;
    std::list<std::tuple<uint32_t, uint32_t, BaseEdge>> result;
    BaseEdge last_edge = edge;
    uint32_t triangle_to_check;
    Triangles surface_triangles = grid_ptr_->get_triangles();

    result.push_back({find_triangles_by_edge(edge).first, find_triangles_by_edge(edge).second, edge});
    triangle_to_check = find_triangles_by_edge(edge).first;

    while (was_updated) {
        was_updated = false;

        //Checking if the edge separeting the previous pair is the longest in current triangle
        if (find_longest_edge(surface_triangles.find(triangle_to_check)->second) != last_edge) {

            last_edge = find_longest_edge(surface_triangles.find(triangle_to_check)->second);

            //Pushing back a pair of triangles separated by the longest edge
            result.push_back({find_triangles_by_edge(last_edge).first, find_triangles_by_edge(last_edge).second, last_edge});

            //Adding a new triangle to the check list
            if (find_triangles_by_edge(last_edge).first == triangle_to_check) {
                triangle_to_check = find_triangles_by_edge(last_edge).second;
                was_updated = true;
            }
            else {
                triangle_to_check = find_triangles_by_edge(last_edge).first;
                was_updated = true;
            }
        }
    }

    triangle_to_check = find_triangles_by_edge(edge).second;
    last_edge = edge;

    while (was_updated) {
        was_updated = false;

        //Checking if the edge separeting the previous pair is the longest in current triangle
        if (find_longest_edge(surface_triangles.find(triangle_to_check)->second) != last_edge) {

            last_edge = find_longest_edge(surface_triangles.find(triangle_to_check)->second);

            //Pushing back a pair of triangles separated by the longest edge
            result.push_back({find_triangles_by_edge(last_edge).first, find_triangles_by_edge(last_edge).second, last_edge});

            //Adding a new triangle to the check list
            if (find_triangles_by_edge(last_edge).first == triangle_to_check) {
                triangle_to_check = find_triangles_by_edge(last_edge).second;
                was_updated = true;
            }
            else {
                triangle_to_check = find_triangles_by_edge(last_edge).first;
                was_updated = true;
            }
        }
    }

    result.reverse();
    result.unique();

    return result;
}

void DelaunayADF3D::generate_inner_points() {
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::generate_inner_points()";
    const std::string CURRENT_STEP_ = "Insertion Front correction";

    //Generating initial front from boundary surface mesh
    std::set<uint32_t> unprocessed_tetrahedra;
    for (const auto& t : grid_ptr_->get_triangles())
        insertion_front_.push_back(t);


    for (const auto& t : grid_ptr_->get_tetrahedrons())
        unprocessed_tetrahedra.insert(t.first);

    Nodes& nodes = grid_ptr_->get_nodes();
    BaseTriangle triangle_to_check;
    BaseTetrahedron current_tetrahedron;
    bool removed = false;

    //Generating list of tetrahedrons in each zone
    for (const auto& t : insertion_front_) {
        std::cout << "Amount of tetrahedra left in the unprocessed group is " + std::to_string(unprocessed_tetrahedra.size()) << std::endl;
        for (auto& u : unprocessed_tetrahedra) {
            current_tetrahedron = grid_ptr_->get_tetrahedrons().find(u)->second;
            for (uint32_t i = 0; i < 4; ++i) {
                triangle_to_check = BaseTriangle(current_tetrahedron.node_ids_[(i + 1) % 4], current_tetrahedron.node_ids_[(i + 2) % 4], current_tetrahedron.node_ids_[(i + 3) % 4]);
                if (t.second == triangle_to_check) {
                    if (is_visible(t.second, nodes.find(current_tetrahedron.node_ids_[i])->second)) {
                        zonal_division_.insert({u, outer_area_});
                        zonal_division_.insert({neighbors_.find(u)->second[i], inner_area_});
                    }
                    else {
                        zonal_division_.insert({u, inner_area_});
                        zonal_division_.insert({neighbors_.find(u)->second[i], outer_area_});
                    }

                    unprocessed_tetrahedra.erase(neighbors_.find(u)->second[i]);
                    removed = true;
                }
            }
            if (removed) {
                unprocessed_tetrahedra.erase(u);
                removed = false;
                break;
            }
        }
    }

    /*
    UnstructuredGrid a;
    uint32_t saaas = 1;
    a.nodes_ = grid_ptr_->get_nodes();
    a.surface_triangles_ = grid_ptr_->get_triangles();

    for (auto& t : grid_ptr_->get_tetrahedrons()) {
        if (zonal_division_.find(t.first) != zonal_division_.end()) {
            a.tetrahedrons_.insert({saaas, t.second});
            ++saaas;
        }
    }
    MeshUnloader3D unloader;
    unloader.unload("../MyDiploma/mesh_files/initial_front_separation.msh", a);

    UnstructuredGrid b;
    uint32_t sbs = 1;
    b.nodes_ = grid_ptr_->get_nodes();
    b.surface_triangles_ = grid_ptr_->get_triangles();

    for (auto& t : grid_ptr_->get_tetrahedrons()) {
        if (zonal_division_.find(t.first) == zonal_division_.end()) {
            b.tetrahedrons_.insert({sbs, t.second});
            ++sbs;
        }
    }
    unloader.unload("../MyDiploma/mesh_files/initial_front_separation_other_tetrahedra.msh", b);
    */

/* //OLD VERSION OF THIS ALGORITHM
    removed = false;
    while (!unprocessed_tetrahedra.empty()) {
        std::cout << "Amount of tetrahedra left in the unprocessed group is " + std::to_string(unprocessed_tetrahedra.size()) << std::endl;
        for (auto& u : unprocessed_tetrahedra) {
            for (uint32_t i = 0; i < 4; ++i) {
                if (zonal_division_.find(neighbors_.find(u)->second[i]) != zonal_division_.end()) {
                    zonal_division_.insert({u, zonal_division_.find(neighbors_.find(u)->second[i])->second});
                    removed = true;
                }
            }
            if (removed) {
                unprocessed_tetrahedra.erase(u);
                removed = false;
                break;
            }
        }
    } */


    while (!unprocessed_tetrahedra.empty()) {
        std::cout << "Amount of tetrahedra left in the unprocessed group is " + std::to_string(unprocessed_tetrahedra.size()) << std::endl;
        for (auto& u : unprocessed_tetrahedra) {
            for (uint32_t i = 0; i < 4; ++i) {
                if (zonal_division_.find(neighbors_.find(u)->second[i]) != zonal_division_.end()) {
                    zonal_division_.insert({u, zonal_division_.find(neighbors_.find(u)->second[i])->second});
                    for (uint32_t j = 0; j < 4; ++j) {
                        if (i != j)
                            zonal_division_.insert({neighbors_.find(u)->second[j], zonal_division_.find(neighbors_.find(u)->second[i])->second});
                        removed = true;
                    }
                    if (removed)
                        break;
                }
            }
            if (removed) {
                for (uint32_t i = 0; i < 4; ++i) {
                    unprocessed_tetrahedra.erase(neighbors_.find(u)->second[i]);
                }
                unprocessed_tetrahedra.erase(u);
                removed = false;
                break;
            }
        }
    }

//----------------------------------------------------------
//    EVERYTHING ABOVE WORKS JUST FINE DON'T TOUCH IT
//----------------------------------------------------------

    while (!insertion_front_.empty()) {
        //Generate a list of candidates
        BaseTriangle current_triangle = insertion_front_.begin()->second;
        BaseTetrahedron current_tetrahedron;
        uint32_t current_tetra_id = 0;
        //uint32_t old_tetrahedron_vertex;
        Tetrahedrons& tetrahedrons = grid_ptr_->get_tetrahedrons();
        std::list<uint32_t> free_front_triangle_ids;
        uint32_t greatest_front_triangle_id = greatest_triangle_id_;
        Nodes candidate_list = generate_candidate_list(nodes.find(current_triangle.node_ids_[0])->second,
                                                       nodes.find(current_triangle.node_ids_[1])->second,
                                                       nodes.find(current_triangle.node_ids_[2])->second);

        std::pair<uint32_t, uint32_t> buff_tetra = find_adjacent_tetrahedra(current_triangle, grid_ptr_->get_tetrahedrons());

        if (zonal_division_.find(buff_tetra.first)->second == inner_area_) {
            current_tetrahedron = tetrahedrons.find(buff_tetra.first)->second;
            current_tetra_id = buff_tetra.first;
        }
        else if (zonal_division_.find(buff_tetra.second)->second == inner_area_) {
            current_tetrahedron = tetrahedrons.find(buff_tetra.second)->second;
            current_tetra_id = buff_tetra.second;
        }
        else {
            std::cout << "???" << std::endl;
        }

        //Find the best candidate
        std::vector<double> resulting_quality;
        std::vector<uint32_t> tested_node_id;
        std::vector<uint32_t> erase_list;
        DelaunayCore3D core;
        for (const auto& c : candidate_list) {
            nodes.insert({greatest_node_id_ + 1, c.second});
            core.clear();

            uint32_t base_id = get_base_elemnt_id(c.second);
            if (zonal_division_.find(base_id)->second != inner_area_) {
                break;
            }


            generate_core(core, c.second);

            for (const auto& t : core.tetrahedrons_to_delete_)
                if (zonal_division_.find(t)->second != inner_area_)
                    erase_list.push_back(t);

            for (const auto& e : erase_list)
                    core.tetrahedrons_to_delete_.erase(e);

            generate_core_front(core);
            correct_core(core, c.second);

            Tetrahedrons new_tetrahedrons;
            uint32_t new_tetrahedron_id = 0;

            for (const auto& t : core.core_front_) {
                ++new_tetrahedron_id;

                //Creating and inserting a new tetrahedron
                Tetrahedron new_tetrahedron(new_tetrahedron_id, BaseTetrahedron(std::get<0>(t).node_ids_[0], std::get<0>(t).node_ids_[1], std::get<0>(t).node_ids_[2], greatest_node_id_ + 1));
                new_tetrahedrons.insert(std::move(new_tetrahedron));
            }

            double old_worst_elem = 1.;
            double new_worst_elem = 1.;
            double old_sum_elem = 0.;
            double new_sum_elem = 0.;
            double buff_quality = 0;
/*
            for (const auto& c : core.tetrahedrons_to_delete_) {
                buff_quality = tetrahedron_quality(nodes.find(tetrahedrons.find(c)->second.node_ids_[0])->second,
                                                   nodes.find(tetrahedrons.find(c)->second.node_ids_[1])->second,
                                                   nodes.find(tetrahedrons.find(c)->second.node_ids_[2])->second,
                                                   nodes.find(tetrahedrons.find(c)->second.node_ids_[3])->second, rho_);
                if (old_worst_elem > buff_quality)
                    old_worst_elem = buff_quality;
            }

            for (const auto& n : new_tetrahedrons) {
                buff_quality = tetrahedron_quality(nodes.find(n.second.node_ids_[0])->second,
                                                   nodes.find(n.second.node_ids_[1])->second,
                                                   nodes.find(n.second.node_ids_[2])->second,
                                                   nodes.find(n.second.node_ids_[3])->second, rho_);
                if (new_worst_elem > buff_quality)
                    new_worst_elem = buff_quality;
            }


            resulting_quality.push_back(new_worst_elem / old_worst_elem);
*/

            for (const auto& c : core.tetrahedrons_to_delete_) {
                buff_quality = tetrahedron_quality(nodes.find(tetrahedrons.find(c)->second.node_ids_[0])->second,
                                                    nodes.find(tetrahedrons.find(c)->second.node_ids_[1])->second,
                                                    nodes.find(tetrahedrons.find(c)->second.node_ids_[2])->second,
                                                    nodes.find(tetrahedrons.find(c)->second.node_ids_[3])->second, rho_);

                old_sum_elem += buff_quality;
                if (old_worst_elem > buff_quality)
                    old_worst_elem = buff_quality;
            }

            old_worst_elem = old_worst_elem * old_sum_elem / core.tetrahedrons_to_delete_.size();
            buff_quality = 0;

            for (const auto& n : new_tetrahedrons) {
                buff_quality = tetrahedron_quality(nodes.find(n.second.node_ids_[0])->second,
                                                    nodes.find(n.second.node_ids_[1])->second,
                                                    nodes.find(n.second.node_ids_[2])->second,
                                                    nodes.find(n.second.node_ids_[3])->second, rho_);

                new_sum_elem += buff_quality;

                if (new_worst_elem > buff_quality)
                    new_worst_elem = buff_quality;
            }


            new_worst_elem = new_worst_elem * new_sum_elem / new_tetrahedrons.size();
            resulting_quality.push_back(new_worst_elem / old_worst_elem);

            tested_node_id.push_back(c.first);
            nodes.erase(greatest_node_id_ + 1);
        }

        double best_ratio = 0;
        uint32_t resulting_node_id = 0;
        for (uint32_t i = 0; i < resulting_quality.size(); ++i) {
            if (best_ratio < resulting_quality[i]) {
                resulting_node_id = tested_node_id[i];
                best_ratio = resulting_quality[i];
            }
        }

        //If all points made the mesh worse, just updating the front
        if (best_ratio > 1) {
            //Insert candidate and update front
            nodes.insert({greatest_node_id_ + 1, candidate_list.find(resulting_node_id)->second});
            ++greatest_node_id_;

            core.clear();

            generate_core(core, nodes.find(greatest_node_id_)->second);

            for (const auto& t : core.tetrahedrons_to_delete_)
                if (zonal_division_.find(t)->second != inner_area_)
                    erase_list.push_back(t);

            for (const auto& e : erase_list)
                    core.tetrahedrons_to_delete_.erase(e);

            generate_core_front(core);
            correct_core(core, nodes.find(greatest_node_id_)->second);

            triangulate_core(core, *nodes.find(greatest_node_id_), true);

            std::pair<uint32_t, uint32_t> buff_tetra = find_adjacent_tetrahedra(current_triangle, grid_ptr_->get_tetrahedrons());

            if (zonal_division_.find(buff_tetra.first)->second == inner_area_) {
                current_tetrahedron = tetrahedrons.find(buff_tetra.first)->second;
                current_tetra_id = buff_tetra.first;
            }
            else if (zonal_division_.find(buff_tetra.second)->second == inner_area_) {
                current_tetrahedron = tetrahedrons.find(buff_tetra.second)->second;
                current_tetra_id = buff_tetra.second;
            }
            else {
                std::cout << "???" << std::endl;
            }
        }

        zonal_division_.find(current_tetra_id)->second = meshed_area_;

/*        UnstructuredGrid a;
        uint32_t b = 1;
        uint32_t saaas = 1;
        uint32_t s = 1000;
        a.nodes_ = grid_ptr_->get_nodes();
        for (const auto& c : candidate_list) {
            a.nodes_.insert({s, c.second});
            ++s;
        }
        for (auto& t : insertion_front_) {
            a.surface_triangles_.insert({b, t.second});
            ++b;
        }

        for (auto& t : grid_ptr_->get_tetrahedrons()) {
            if (zonal_division_.find(t.first) != zonal_division_.end()) {
                if (zonal_division_.find(t.first)->second == 1) {
                    a.tetrahedrons_.insert({saaas, t.second});
                    ++saaas;
                }
            }
        }
        MeshUnloader3D unloader;
        unloader.unload("../MyDiploma/mesh_files/slomalos5.msh", a);
        std::cout << "Oopsie Doopsie" << std::endl;*/

        uint32_t new_front_id;
        std::vector<Triangle> to_delete;
        for (uint8_t i = 0; i < 4; ++i) {
            if (zonal_division_.find(neighbors_.find(current_tetra_id)->second[i])->second == inner_area_) {

                if (!free_front_triangle_ids.empty()) {
                    new_front_id = free_front_triangle_ids.front();
                    free_front_triangle_ids.pop_front();
                }
                else {
                    ++greatest_front_triangle_id;
                    new_front_id = greatest_triangle_id_;
                }
                //With how we defined an orientation of the tetrahedron, the correct numeration of triangles is:
                //Opposite to node 1: {2, 4, 3}
                //Opposite to node 2: {1, 3, 4}
                //Opposite to node 3: {1, 4, 2}
                //Opposite to node 4: {1, 2, 3}
                switch (i) {
                    case 0: {
                        insertion_front_.push_back({new_front_id, BaseTriangle(current_tetrahedron.node_ids_[1],
                                                                               current_tetrahedron.node_ids_[3],
                                                                               current_tetrahedron.node_ids_[2])});
                        break;
                    }
                    case 1: {
                        insertion_front_.push_back({new_front_id, BaseTriangle(current_tetrahedron.node_ids_[0],
                                                                               current_tetrahedron.node_ids_[2],
                                                                               current_tetrahedron.node_ids_[3])});
                        break;
                    }
                    case 2: {
                        insertion_front_.push_back({new_front_id, BaseTriangle(current_tetrahedron.node_ids_[0],
                                                                               current_tetrahedron.node_ids_[3],
                                                                               current_tetrahedron.node_ids_[1])});
                        break;
                    }
                    case 3: {
                        insertion_front_.push_back({new_front_id, BaseTriangle(current_tetrahedron.node_ids_[0],
                                                                               current_tetrahedron.node_ids_[1],
                                                                               current_tetrahedron.node_ids_[2])});
                        break;
                    }
                    default:
                        throw GeneratorException(302, __LINE__, __FILE__, CURRENT_STEP_, CURRENT_FUNCTION_, "Fatal error. If you are seeing this error - the problem is in triangle insertion algorithm.");
                }
            }
            else {
                for (auto& t : insertion_front_) {
                    if (t.second == BaseTriangle(current_tetrahedron.node_ids_[(i + 1) % 4], current_tetrahedron.node_ids_[(i + 2) % 4], current_tetrahedron.node_ids_[(i + 3) % 4])) {
                        to_delete.push_back(t);
                        break;
                    }
                }

            for (auto& td : to_delete) {
                    insertion_front_.remove(td);
                    free_front_triangle_ids.push_back(td.first);
                }
            to_delete.clear();
            }
        }

        std::cout << "Insertion front has " + std::to_string(insertion_front_.size()) + " triangles" << std::endl;
    }

            UnstructuredGrid result_test;
            uint32_t tetra = 1;
            result_test.nodes_ = grid_ptr_->get_nodes();
            //result_test.edges_ = grid_ptr_->get_edges();
            //result_test.surface_triangles_ = grid_ptr_->get_triangles();

            for (auto& t : grid_ptr_->get_tetrahedrons()) {
                if (zonal_division_.find(t.first) != zonal_division_.end()) {
                    if (zonal_division_.find(t.first)->second == 1) {
                        result_test.tetrahedrons_.insert({tetra, t.second});
                        ++tetra;
                    }
                }
            }
            MeshUnloader3D unloader;
            unloader.unload("../MyDiploma/mesh_files/resulting_mesh.msh", result_test);
            std::cout << "Oopsie Doopsie" << std::endl;
}

Nodes DelaunayADF3D::generate_candidate_list(const BaseNode& a, const BaseNode& b, const BaseNode& c) {
    Nodes result;
    BaseNode middle_node({(a.coords_[0] + b.coords_[0] + c.coords_[0]) / 3., (a.coords_[1] + b.coords_[1] + c.coords_[1]) / 3., (a.coords_[2] + b.coords_[2] + c.coords_[2]) / 3.});
    BaseNode x_test_node({(a.coords_[0] + b.coords_[0] + c.coords_[0]) * 2. / 5., (a.coords_[1] + b.coords_[1] + c.coords_[1]) / 5., (a.coords_[2] + b.coords_[2] + c.coords_[2]) / 5.});
    BaseNode y_test_node({(a.coords_[0] + b.coords_[0] + c.coords_[0]) / 5., (a.coords_[1] + b.coords_[1] + c.coords_[1]) * 2. / 5., (a.coords_[2] + b.coords_[2] + c.coords_[2]) / 5.});
    BaseNode z_test_node({(a.coords_[0] + b.coords_[0] + c.coords_[0]) / 5., (a.coords_[1] + b.coords_[1] + c.coords_[1]) / 5., (a.coords_[2] + b.coords_[2] + c.coords_[2]) * 2. / 5.});
    BaseNode new_node;
    BaseNode normal = calculate_normal(a, b, c);
    uint32_t node_num = 1;
    double suggested_height = std::sqrt(2. * (std::pow((node_distance(a, b)), 2.) + std::pow((node_distance(b, c)), 2.) + std::pow((node_distance(a, c)), 2.)) / 9.);

    //Reversing the normal if orientation demands so
    if (counter_clockwise_orientation_)
        for (uint8_t i = 0; i < 3; ++i)
            normal.coords_[i] *= -1.;

    for (uint8_t i = 0; i < 9; ++i) {
        new_node.coords_[0] = middle_node.coords_[0] + normal.coords_[0] * suggested_height * (0.9 + 0.05 * i);
        new_node.coords_[1] = middle_node.coords_[1] + normal.coords_[1] * suggested_height * (0.9 + 0.05 * i);
        new_node.coords_[2] = middle_node.coords_[2] + normal.coords_[2] * suggested_height * (0.9 + 0.05 * i);
        result.insert({node_num, new_node});
        ++node_num;
    }

/*
        new_node.coords_[0] = x_test_node.coords_[0] + normal.coords_[0] * suggested_height * 0.5;
        new_node.coords_[1] = x_test_node.coords_[1] + normal.coords_[1] * suggested_height * 0.5;
        new_node.coords_[2] = x_test_node.coords_[2] + normal.coords_[2] * suggested_height * 0.5;
        result.insert({node_num, new_node});
        ++node_num;

        new_node.coords_[0] = y_test_node.coords_[0] + normal.coords_[0] * suggested_height * 0.5;
        new_node.coords_[1] = y_test_node.coords_[1] + normal.coords_[1] * suggested_height * 0.5;
        new_node.coords_[2] = y_test_node.coords_[2] + normal.coords_[2] * suggested_height * 0.5;
        result.insert({node_num, new_node});
        ++node_num;

        new_node.coords_[0] = y_test_node.coords_[0] + normal.coords_[0] * suggested_height * 0.5;
        new_node.coords_[1] = y_test_node.coords_[1] + normal.coords_[1] * suggested_height * 0.5;
        new_node.coords_[2] = y_test_node.coords_[2] + normal.coords_[2] * suggested_height * 0.5;
        result.insert({node_num, new_node});
        ++node_num;
*/
    return result;
}


//This is an untested function, that can raise our efficiency by quite a bit.
//Too bad we aren't using it
void DelaunayADF3D::build_surface_neighbors() {
    surface_neighbors_.clear();
    for (auto& t : grid_ptr_->get_triangles())
        surface_neighbors_.insert({t.first, {0, 0, 0}});

    // Инициализируем массив P нулями
    std::vector<uint32_t> P(greatest_node_id_ + 1, 0);
    Triangles triangles = grid_ptr_->get_triangles();

    for (uint32_t i = 1; i <= greatest_triangle_id_; ++i) {
        auto t = triangles.find(i)->second;
        for (uint32_t j = 0; j < 3; ++j) {
            uint32_t j1 = t.node_ids_[(j + 1) % 3];
            uint32_t j2 = t.node_ids_[(j + 2) % 3];
            uint32_t k = std::min(j1, j2);
            P[k] += 1;
        }
    }

    for (uint32_t k = 2; k < greatest_node_id_ + 1; ++k)
        P[k] += P[k - 1];

    uint32_t n = P[greatest_node_id_];

    std::vector<uint32_t> Q(n, 0);
    for (uint32_t i = 1; i <= greatest_triangle_id_; ++i) {
        auto t = triangles.find(i)->second;

        for (uint32_t j = 0; j < 3; ++j) {
            uint32_t j1 = t.node_ids_[(j + 1) % 3];
            uint32_t j2 = t.node_ids_[(j + 2) % 3];
            uint32_t k1 = std::min(j1, j2);
            uint32_t k2 = std::max(j1, j2);
            uint32_t m = P[k1];
            while (true) {
                m--;
                if (Q[m] == 0) {
                    Q[m] = 3 * i + j;
                    surface_neighbors_.find(i)->second[j] = 0;
                    break;
                }
                else {
                    uint32_t i_star = Q[m] / 3;
                    if (i == i_star) {
                        surface_neighbors_.find(i)->second[j] = 0;
                        continue;
                    }

                    auto tp = triangles.find(i_star)->second;

                    uint32_t j_star = Q[m] % 3;

                    uint32_t j1_star = tp.node_ids_[(j_star+1) % 3];
                    uint32_t j2_star = tp.node_ids_[(j_star+2) % 3];

                    uint32_t k2_star = std::max(j1_star, j2_star);

                    if (k2 != k2_star) continue;
                    else {
                        surface_neighbors_.find(i)->second[j] = i_star;
                        surface_neighbors_.find(i_star)->second[j_star] = i;
                        break;
                    }
                }
            }
        }
    }
}

void DelaunayADF3D::quality_checker() {
    std::vector<uint32_t> result(10, 0);
    Nodes& nodes = grid_ptr_->get_nodes();

    for (const auto& t : grid_ptr_->get_tetrahedrons()) {
        if (zonal_division_.find(t.first) != zonal_division_.end()) {
            if (zonal_division_.find(t.first)->second == 1) {
                double quality = tetrahedron_quality(nodes.find(t.second.node_ids_[0])->second,
                                                     nodes.find(t.second.node_ids_[1])->second,
                                                     nodes.find(t.second.node_ids_[2])->second,
                                                     nodes.find(t.second.node_ids_[3])->second, rho_);

                if ((0. <= quality) && (quality < 0.1)) {
                    result[0]++;
                }
                if ((0.1 <= quality) && (quality < 0.2)) {
                    result[1]++;
                }
                if ((0.2 <= quality) && (quality < 0.3)) {
                    result[2]++;
                }
                if ((0.3 <= quality) && (quality < 0.4)) {
                    result[3]++;
                }
                if ((0.4 <= quality) && (quality < 0.5)) {
                    result[4]++;
                }
                if ((0.5 <= quality) && (quality < 0.6)) {
                    result[5]++;
                }
                if ((0.6 <= quality) && (quality < 0.7)) {
                    result[6]++;
                }
                if ((0.7 <= quality) && (quality < 0.8)) {
                    result[7]++;
                }
                if ((0.8 <= quality) && (quality < 0.9)) {
                    result[8]++;
                }
                if ((0.9 <= quality) && (quality < 1.)) {
                    result[9]++;
                }
            }
        }
    }

    for (const auto& q : result)
        std::cout << q << ", ";

    std::cout << std::endl;


    /*
    std::cout << "Elements with quality between 0. and 0.1 " << result[0] << std::endl <<
                 "Elements with quality between 0.1 and 0.2 " << result[1] << std::endl <<
                 "Elements with quality between 0.2 and 0.3 " << result[2] << std::endl <<
                 "Elements with quality between 0.3 and 0.4 " << result[3] << std::endl <<
                 "Elements with quality between 0.4 and 0.5 " << result[4] << std::endl <<
                 "Elements with quality between 0.5 and 0.6 " << result[5] << std::endl <<
                 "Elements with quality between 0.6 and 0.7 " << result[6] << std::endl <<
                 "Elements with quality between 0.7 and 0.8 " << result[7] << std::endl <<
                 "Elements with quality between 0.8 and 0.9 " << result[8] << std::endl <<
                 "Elements with quality between 0.9 and 1. " << result[9] << std::endl;
                 */
}


void DelaunayADF3D::problem_generator() {
    Nodes& nodes = grid_ptr_->get_nodes();
    uint8_t rep = 0;
    for (auto& t : grid_ptr_->get_triangles()) {
        rep = 0;
        for (uint32_t i = 0; i < 3; ++i) {
            if (almost_equal(nodes.find(t.second.node_ids_[i])->second.coords_[0], -0.5)) {
                ++rep;
            }
        }
        if (rep == 3) {
            t.second.elementary_ = 2;
        }
        else {
            t.second.elementary_ = 13;
        }
    }
}

void DelaunayADF3D::generate_delaunay_only() {
    const std::string CURRENT_FUNCTION_ = "DelaunayADF3D::generate_inner_points()";
    const std::string CURRENT_STEP_ = "Insertion Front correction";

    //Generating initial front from boundary surface mesh
    std::set<uint32_t> unprocessed_tetrahedra;
    for (const auto& t : grid_ptr_->get_triangles())
        insertion_front_.push_back(t);


    for (const auto& t : grid_ptr_->get_tetrahedrons())
        unprocessed_tetrahedra.insert(t.first);

    Nodes& nodes = grid_ptr_->get_nodes();
    BaseTriangle triangle_to_check;
    BaseTetrahedron current_tetrahedron;
    bool removed = false;

    //Generating list of tetrahedrons in each zone
    for (const auto& t : insertion_front_) {
        std::cout << "Amount of tetrahedra left in the unprocessed group is " + std::to_string(unprocessed_tetrahedra.size()) << std::endl;
        for (auto& u : unprocessed_tetrahedra) {
            current_tetrahedron = grid_ptr_->get_tetrahedrons().find(u)->second;
            for (uint32_t i = 0; i < 4; ++i) {
                triangle_to_check = BaseTriangle(current_tetrahedron.node_ids_[(i + 1) % 4], current_tetrahedron.node_ids_[(i + 2) % 4], current_tetrahedron.node_ids_[(i + 3) % 4]);
                if (t.second == triangle_to_check) {
                    if (is_visible(t.second, nodes.find(current_tetrahedron.node_ids_[i])->second)) {
                        zonal_division_.insert({u, outer_area_});
                        zonal_division_.insert({neighbors_.find(u)->second[i], inner_area_});
                    }
                    else {
                        zonal_division_.insert({u, inner_area_});
                        zonal_division_.insert({neighbors_.find(u)->second[i], outer_area_});
                    }

                    unprocessed_tetrahedra.erase(neighbors_.find(u)->second[i]);
                    removed = true;
                }
            }
            if (removed) {
                unprocessed_tetrahedra.erase(u);
                removed = false;
                break;
            }
        }
    }

    while (!unprocessed_tetrahedra.empty()) {
        std::cout << "Amount of tetrahedra left in the unprocessed group is " + std::to_string(unprocessed_tetrahedra.size()) << std::endl;
        for (auto& u : unprocessed_tetrahedra) {
            for (uint32_t i = 0; i < 4; ++i) {
                if (zonal_division_.find(neighbors_.find(u)->second[i]) != zonal_division_.end()) {
                    zonal_division_.insert({u, zonal_division_.find(neighbors_.find(u)->second[i])->second});
                    for (uint32_t j = 0; j < 4; ++j) {
                        if (i != j)
                            zonal_division_.insert({neighbors_.find(u)->second[j], zonal_division_.find(neighbors_.find(u)->second[i])->second});
                        removed = true;
                    }
                    if (removed)
                        break;
                }
            }
            if (removed) {
                for (uint32_t i = 0; i < 4; ++i) {
                    unprocessed_tetrahedra.erase(neighbors_.find(u)->second[i]);
                }
                unprocessed_tetrahedra.erase(u);
                removed = false;
                break;
            }
        }
    }


    missing_connections_list_.clear();
    inner_edges_.clear();

    missing_connections_list_ = std::vector<std::set<uint32_t>>(grid_ptr_->get_nodes().size() + 1);

    for (const auto& t : grid_ptr_->get_tetrahedrons())
        if (zonal_division_.find(t.first) != zonal_division_.end())
            if (zonal_division_.find(t.first)->second == inner_area_)
                for (uint32_t i = 0; i < 4; ++i)
                    for (uint32_t j = 0; j < 4; ++j)
                        if (i != j)
                            missing_connections_list_[t.second.node_ids_[i]].insert(t.second.node_ids_[j]);

    for (const auto& t : grid_ptr_->get_triangles())
        for (uint32_t i = 0; i < 3; ++i)
            for (uint32_t j = 0; j < 3; ++j)
                if (i != j)
                    missing_connections_list_[t.second.node_ids_[i]].erase(t.second.node_ids_[j]);

    for (uint32_t i = 1; i < missing_connections_list_.size(); ++i)
        if (!missing_connections_list_[i].empty())
            for (const auto& j : missing_connections_list_[i])
                if (i < j)
                    inner_edges_.push_back(BaseEdge(i, j));

    std::vector<BaseNode> new_nodes;
    uint32_t divisions = 0;
    for (const auto& i : inner_edges_) { // cube 1. / 6.5 ;
        divisions = static_cast<uint32_t>(node_distance(nodes.find(i.node_ids_[0])->second, nodes.find(i.node_ids_[1])->second) /  (1. / 13.));
        if (divisions >= 2) {
            for (uint32_t j = 1; j < divisions; ++j) {
                BaseNode new_node;
                for (uint32_t k = 0; k < 3; ++k)
                    new_node.coords_[k] = nodes.find(i.node_ids_[0])->second.coords_[k] + (nodes.find(i.node_ids_[1])->second.coords_[k] - nodes.find(i.node_ids_[0])->second.coords_[k]) * j / divisions;


                new_nodes.push_back(std::move(new_node));
            }
        }
    }

    bool is_bad = false;

    for (uint32_t i = 0; i < new_nodes.size(); ++i) {
        is_bad = false;
        for (uint32_t j = i + 1; j < new_nodes.size(); ++j) // cube 1. /12. ;
            if (node_distance(new_nodes[i], new_nodes[j]) < 1. /20.) {
                is_bad = true;
                break;
            }

        if (!is_bad) {
            nodes.insert({greatest_node_id_ + 1, new_nodes[i]});
            insert_node({greatest_node_id_ + 1, new_nodes[i]});
            ++greatest_node_id_;
            std::cout << "Inserted node with id " << std::to_string(greatest_node_id_) << std::endl;
        }
    }


    UnstructuredGrid result_test;
    bool upload = true;
    uint32_t tetra = 1;
    result_test.nodes_ = grid_ptr_->get_nodes();
    //result_test.edges_ = grid_ptr_->get_edges();
    //result_test.surface_triangles_ = grid_ptr_->get_triangles();

    for (auto& t : grid_ptr_->get_tetrahedrons()) {
        upload = true;
        for (uint32_t i = 0; i < 4; ++i)
            if (t.second.node_ids_[i] < 9)
                upload = false;

        if (upload) {
            result_test.tetrahedrons_.insert({tetra, t.second});
            ++tetra;
        }

    }
    MeshUnloader3D unloader;
    unloader.unload("../MyDiploma/mesh_files/resulting_mesh_vulpes.msh", result_test);
    std::cout << "Oopsie Doopsie" << std::endl;

}

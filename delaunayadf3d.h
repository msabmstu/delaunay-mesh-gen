#ifndef DELAUNAYADF3D_H
#define DELAUNAYADF3D_H

#include <abstractgenerator.h>
#include <memory>
#include <exceptions.h>
#include <list>
#include <stack>

class DelaunayADF3D : public AbstractGenerator
{
private:
    bool counter_clockwise_orientation_;
    uint8_t outer_area_;
    uint8_t inner_area_;
    uint8_t meshed_area_;
    uint32_t greatest_node_id_;
    uint32_t greatest_edge_id_;
    uint32_t greatest_triangle_id_;
    uint32_t greatest_tetrahedron_id_;
    std::shared_ptr<UnstructuredGrid> grid_ptr_;
    TetraNeighbors neighbors_;
    TriangleNeighbors surface_neighbors_;
    std::set<uint32_t> free_tetrahedron_ids_;
    //List of edges on the surface in the initial mesh, after the node insertion procedure. It is moved 1 spce down, which makes a vertex with ID 56 be #55 in the vector
    std::vector<std::set<uint32_t>> missing_connections_list_;
    std::list<BaseEdge> missing_edges_;
    std::list<BaseEdge> inner_edges_;
    std::list<Triangle> insertion_front_;
    std::unordered_map<uint32_t, uint8_t> zonal_division_;
    std::function<double(const BaseNode&)> rho_;

    ///Generating convex hull covering the figure. This function changes the IDs of nodes! (It's needed to simplify the generation later)
    void generate_convex_hull();

    ///Inserting 1 node to existing delaunay tetrahedrezation
    void insert_node(const Node& new_node);

    ///Finding edges of the CORE and adding them
    void generate_core(DelaunayCore3D& core, const BaseNode& new_node) const;

    void correct_core(DelaunayCore3D& core, const BaseNode& new_node) const;

    void generate_core_front(DelaunayCore3D& core) const;

    ///Deleting tetrahedrons from the CORE and correcting a new one
    void triangulate_core(const DelaunayCore3D& core, const Node& new_node, bool zonal_division_active = false);

    ///Generating the list of edges of initial surface mesh, that are missing after node insertion
    void find_missing_edges();

    uint32_t get_base_elemnt_id(const BaseNode& node_to_insert) const;

    ///Retruns IDs of both adjacent tetrahedrons, if the search fails - returns (0, 0)
    std::pair<uint32_t, uint32_t> find_adjacent_tetrahedra(const BaseTriangle& triangle, const Tetrahedrons& tetrahedrons) const;

    ///Returns TRUE if point is OUTSIDE of the sphere, FALSE - otherwise
    bool is_delaunay(uint32_t tetrahedron_id, const BaseNode& node_to_check) const;

    ///Returns TRUE if triangle is visible from the point by checking signed volume
    bool is_visible(const BaseTriangle& triangle, const BaseNode& node) const;


    void recover_boundaries();

    ///Changes the tetrahedron ID
    void tetra_id_change(uint32_t swap_from, uint32_t swap_to);

    BaseEdge find_longest_edge(const BaseTriangle& triangle);

    void build_surface_neighbors();

    std::pair<uint32_t, uint32_t> find_triangles_by_edge(const BaseEdge& edge);

    ///Generates the REVERSED list of pairs of triangles in need to be changed
    std::list<std::tuple<uint32_t, uint32_t, BaseEdge>> generate_triangle_queue(BaseEdge& edge);

    void generate_inner_points();

    Nodes generate_candidate_list(const BaseNode &a, const BaseNode &b, const BaseNode &c);

public:
    ///Generates the final grid
    virtual void generate() override;

    ///Initializes generator
    virtual void initialize(std::shared_ptr<UnstructuredGrid> grid_ptr, bool counter_clockwise = true, std::function<double(const BaseNode&)> rho = [](const BaseNode&){return 1;}) override;

    //-------------------------------------------------------------------
    //FUNCTIONS BELOW SHOULD BE MOVED TO PRIVATE SECTION LATER.
    //THEY ARE NOT MOVED THERE YET DUE TO THE DIFFICULTIES OF DEBUGGING.
    //-------------------------------------------------------------------

    void quality_checker();

    void problem_generator();

    void generate_delaunay_only();
};

#endif // DELAUNAYADF3D_H

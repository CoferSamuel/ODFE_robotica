#pragma once
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

enum class NodeType { ROOM, DOOR };

struct GraphNode {
    int id;
    NodeType type;
    std::string name;
    // Position: for ROOM = center, for DOOR = midpoint
    Eigen::Vector2f position;
    // Room-specific data
    float width = 0;
    float length = 0;
    int room_index = -1;  // Index in nominal_rooms vector
    float reference_heading = 0;  // Robot heading when room was first recognized
    int entry_door_id = -1;  // Door used to enter this room (-1 = none/starting room)
    // Door-specific data
    Eigen::Vector2f p1{0, 0};
    Eigen::Vector2f p2{0, 0};
};

class Graph {
public:
    explicit Graph(const std::string& log_path);
    ~Graph();

    // Add nodes - returns node ID
    int add_room(const std::string& name, int room_index, float width, float length, 
                 const Eigen::Vector2f& center);
    int add_door(const std::string& name, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);

    // Connect nodes (undirected)
    void connect(int node1_id, int node2_id);

    // Query methods
    bool has_node(int id) const;
    const GraphNode& get_node(int id) const;
    std::vector<int> get_neighbors(int id) const;
    std::vector<int> get_rooms() const;
    std::vector<int> get_doors() const;
    int get_room_node_id(int room_index) const;  // Get node ID for a room index
    bool is_room_in_graph(int room_index) const;
    int node_count() const { return static_cast<int>(nodes_.size()); }
    const std::map<int, GraphNode>& get_nodes() const { return nodes_; }
    
    // Heading constraint for room re-entry
    void set_room_heading(int room_index, float heading);
    float get_room_heading(int room_index) const;
    
    // Door exploration and entry tracking
    void set_entry_door(int room_index, int door_node_id);
    int get_entry_door(int room_index) const;
    bool is_door_explored(int door_node_id) const;  // True if connected to 2 rooms
    std::vector<int> get_unexplored_doors(int room_index) const;  // Doors not connected to 2 rooms
    int get_door_by_position(const Eigen::Vector2f& pos, float tolerance = 500.f) const;

private:
    std::map<int, GraphNode> nodes_;
    std::map<int, std::vector<int>> adjacency_;
    std::map<int, int> room_index_to_node_id_;  // Maps room_index -> node_id
    int next_id_ = 0;
    std::ofstream log_file_;

    void log(const std::string& msg);
};

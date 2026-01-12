#include "graph.h"
#include <sstream>
#include <iomanip>
#include <ctime>

Graph::Graph(const std::string& log_path) {
    log_file_.open(log_path, std::ios::out | std::ios::trunc);
    log("Graph initialized");
}

Graph::~Graph() {
    if (log_file_.is_open()) {
        log("Graph destroyed");
        log_file_.close();
    }
}

void Graph::log(const std::string& msg) {
    if (log_file_.is_open()) {
        auto now = std::time(nullptr);
        log_file_ << std::put_time(std::localtime(&now), "[%H:%M:%S] ") << msg << std::endl;
        log_file_.flush();
    }
}

int Graph::add_room(const std::string& name, int room_index, float width, float length,
                    const Eigen::Vector2f& center) {
    GraphNode node;
    node.id = next_id_++;
    node.type = NodeType::ROOM;
    node.name = name;
    node.position = center;
    node.width = width;
    node.length = length;
    node.room_index = room_index;

    nodes_[node.id] = node;
    adjacency_[node.id] = {};
    room_index_to_node_id_[room_index] = node.id;

    std::ostringstream oss;
    oss << "Added ROOM node: id=" << node.id << " name=\"" << name << "\" room_index=" << room_index
        << " size=(" << width << "x" << length << ") center=(" << center.x() << "," << center.y() << ")";
    log(oss.str());

    return node.id;
}

int Graph::add_door(const std::string& name, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
    GraphNode node;
    node.id = next_id_++;
    node.type = NodeType::DOOR;
    node.name = name;
    node.position = (p1 + p2) / 2.0f;  // Midpoint
    node.p1 = p1;
    node.p2 = p2;

    nodes_[node.id] = node;
    adjacency_[node.id] = {};

    std::ostringstream oss;
    oss << "Added DOOR node: id=" << node.id << " name=\"" << name << "\""
        << " p1=(" << p1.x() << "," << p1.y() << ") p2=(" << p2.x() << "," << p2.y() << ")";
    log(oss.str());

    return node.id;
}

void Graph::connect(int node1_id, int node2_id) {
    if (!has_node(node1_id) || !has_node(node2_id)) {
        log("ERROR: Cannot connect - node does not exist");
        return;
    }

    // Add undirected edge
    adjacency_[node1_id].push_back(node2_id);
    adjacency_[node2_id].push_back(node1_id);

    std::ostringstream oss;
    oss << "Connected: " << nodes_[node1_id].name << " (id=" << node1_id << ") <-> "
        << nodes_[node2_id].name << " (id=" << node2_id << ")";
    log(oss.str());
}

bool Graph::has_node(int id) const {
    return nodes_.find(id) != nodes_.end();
}

const GraphNode& Graph::get_node(int id) const {
    return nodes_.at(id);
}

std::vector<int> Graph::get_neighbors(int id) const {
    if (adjacency_.find(id) == adjacency_.end()) {
        return {};
    }
    return adjacency_.at(id);
}

std::vector<int> Graph::get_rooms() const {
    std::vector<int> rooms;
    for (const auto& [id, node] : nodes_) {
        if (node.type == NodeType::ROOM) {
            rooms.push_back(id);
        }
    }
    return rooms;
}

std::vector<int> Graph::get_doors() const {
    std::vector<int> doors;
    for (const auto& [id, node] : nodes_) {
        if (node.type == NodeType::DOOR) {
            doors.push_back(id);
        }
    }
    return doors;
}

int Graph::get_room_node_id(int room_index) const {
    auto it = room_index_to_node_id_.find(room_index);
    if (it != room_index_to_node_id_.end()) {
        return it->second;
    }
    return -1;  // Not found
}

bool Graph::is_room_in_graph(int room_index) const {
    return room_index_to_node_id_.find(room_index) != room_index_to_node_id_.end();
}

void Graph::set_room_heading(int room_index, float heading) {
    auto it = room_index_to_node_id_.find(room_index);
    if (it != room_index_to_node_id_.end()) {
        nodes_[it->second].reference_heading = heading;
        
        std::ostringstream oss;
        oss << "Set heading for Room_" << room_index << " = " << heading << " rad";
        log(oss.str());
    }
}

float Graph::get_room_heading(int room_index) const {
    auto it = room_index_to_node_id_.find(room_index);
    if (it != room_index_to_node_id_.end()) {
        return nodes_.at(it->second).reference_heading;
    }
    return 0.0f;  // Default heading if room not found
}

void Graph::set_entry_door(int room_index, int door_node_id) {
    auto it = room_index_to_node_id_.find(room_index);
    if (it != room_index_to_node_id_.end()) {
        nodes_[it->second].entry_door_id = door_node_id;
        
        std::ostringstream oss;
        oss << "Set entry door for Room_" << room_index << " = door_id=" << door_node_id;
        log(oss.str());
    }
}

int Graph::get_entry_door(int room_index) const {
    auto it = room_index_to_node_id_.find(room_index);
    if (it != room_index_to_node_id_.end()) {
        return nodes_.at(it->second).entry_door_id;
    }
    return -1;  // No entry door
}

bool Graph::is_door_explored(int door_node_id) const {
    // A door is explored if it has at least 2 neighbors (Parent Room + Linked Door/Room)
    if (adjacency_.find(door_node_id) == adjacency_.end()) return false;
    
    // New logic: Check total connection count (Room + Linked Door = 2)
    return adjacency_.at(door_node_id).size() >= 2;
}

std::vector<int> Graph::get_unexplored_doors(int room_index) const {
    std::vector<int> unexplored;
    int room_id = get_room_node_id(room_index);
    if (room_id < 0) return unexplored;
    
    for (int door_id : get_neighbors(room_id)) {
        if (nodes_.at(door_id).type == NodeType::DOOR && !is_door_explored(door_id)) {
            unexplored.push_back(door_id);
        }
    }
    return unexplored;
}

int Graph::get_door_by_position(const Eigen::Vector2f& pos, float tolerance) const {
    for (const auto& [id, node] : nodes_) {
        if (node.type == NodeType::DOOR) {
            float dist = (node.position - pos).norm();
            if (dist < tolerance) {
                return id;
            }
        }
    }
    return -1;  // Not found
}

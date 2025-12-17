#include "graph.h"
#include <algorithm>

void Graph::add_node(const Node &node) {
  if (nodes.find(node.id) == nodes.end()) {
    nodes[node.id] = node;
    adj[node.id] = {}; // Initialize adjacency list
  }
}

void Graph::add_edge(int id1, int id2) {
  if (nodes.find(id1) != nodes.end() && nodes.find(id2) != nodes.end()) {
    // Check if edge already exists to avoid duplicates
    auto &neighbors1 = adj[id1];
    if (std::find(neighbors1.begin(), neighbors1.end(), id2) ==
        neighbors1.end()) {
      neighbors1.push_back(id2);
    }

    auto &neighbors2 = adj[id2];
    if (std::find(neighbors2.begin(), neighbors2.end(), id1) ==
        neighbors2.end()) {
      neighbors2.push_back(id1);
    }
  }
}

bool Graph::has_node(int id) const { return nodes.find(id) != nodes.end(); }

Node Graph::get_node(int id) const {
  if (nodes.find(id) != nodes.end()) {
    return nodes.at(id);
  }
  return Node(); // Return default/empty node if not found
}

const std::map<int, Node> &Graph::get_nodes() const { return nodes; }

const std::map<int, std::vector<int>> &Graph::get_adj() const { return adj; }

std::vector<int> Graph::get_neighbors(int id) const {
  if (adj.find(id) != adj.end()) {
    return adj.at(id);
  }
  return {};
}

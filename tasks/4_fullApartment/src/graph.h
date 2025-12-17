#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Dense>
#include <any>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Node {
  int id;
  enum Type { ROOM, DOOR } type;
  Eigen::Vector2f pos; // Position for visualization
  std::map<std::string, std::string>
      attributes; // Simple string attributes for now

  Node() = default;
  Node(int id_, Type type_, Eigen::Vector2f pos_)
      : id(id_), type(type_), pos(pos_) {}
};

class Graph {
public:
  Graph() = default;
  ~Graph() = default;

  void add_node(const Node &node);
  void add_edge(int id1, int id2);
  [[nodiscard]] bool has_node(int id) const;
  [[nodiscard]] Node get_node(int id) const;
  [[nodiscard]] const std::map<int, Node> &get_nodes() const;
  [[nodiscard]] const std::map<int, std::vector<int>> &get_adj() const;
  [[nodiscard]] std::vector<int> get_neighbors(int id) const;

private:
  std::map<int, Node> nodes;
  std::map<int, std::vector<int>> adj;
};

#endif // GRAPH_H

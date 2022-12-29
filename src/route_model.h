#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include "model.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

// Class constructor inherits from the Model class provided by IO2D.
class RouteModel : public Model {

public:
  //  Subclass constructor inherits from the struct Node.
  class Node : public Model::Node {
  public:
    Node *parent = nullptr;
    float h_value = std::numeric_limits<float>::max();
    float g_value = 0.0;
    bool visited = false;
    std::vector<Node *> neighbors;

    void FindNeighbors();
    float distance(Node other) const {
      return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
    }

    Node() {}

    // Constructor of RouteModel node from an existing model node.
    Node(int idx, RouteModel *search_model, Model::Node node)
        : Model::Node(node), parent_model(search_model), index(idx) {}

  private:
    int index;
    Node *FindNeighbor(std::vector<int> node_indices);
    RouteModel *parent_model = nullptr;
  };

  // Parse the OSM data in xml into the program.
  RouteModel(const std::vector<std::byte> &xml);
  Node &FindClosestNode(float x, float y);
  auto &SNodes() { return m_Nodes; }
  std::vector<Node> path;

private:
  // Creats a hashmap from the index of node to the neighbor_roads.
  void CreateNodeToRoadHashmap();
  std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
  // All the route_model_nodes that are in the route_model.
  std::vector<Node> m_Nodes;
};

#endif

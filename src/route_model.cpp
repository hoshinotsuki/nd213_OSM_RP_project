#include "route_model.h"
#include <iostream>

// Class constructor with the XML data
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  // Create RouteModel_nodes.
  int counter = 0;
  for (Model::Node node : this->Nodes()) {
    // Store RouteModel_nodes in the m-Nodes vector.
    m_Nodes.emplace_back(Node(counter, this, node));
    counter++;
  }
  CreateNodeToRoadHashmap();
}

// Crate a hashmap from the index of node to the roads that it belongs to.
void RouteModel::CreateNodeToRoadHashmap() {
  for (const Model::Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx : Ways()[road.way].nodes) {
        // Create the HashMap.
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = std::vector<const Model::Road *>();
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

// Return the Closest_node to the target_node in the target_road.
RouteModel::Node *
RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  Node *closest_node = nullptr;
  Node node;

  for (int node_index : node_indices) {
    node = parent_model->SNodes()[node_index];
    // Check if this node is the target_node itself and has been visited.
    if (this->distance(node) != 0 && !node.visited) {
      if (closest_node == nullptr ||
          this->distance(node) < this->distance(*closest_node)) {
        // Replace the closest_node with the current node.
        closest_node = &parent_model->SNodes()[node_index];
      }
    }
  }
  return closest_node;
}

// Put all valid neighbor_node into the vector.
void RouteModel::Node::FindNeighbors() {
  // Look up the roads by the index of node in the node-road hashmap.
  for (auto &road : parent_model->node_to_road[this->index]) {
    // Get the closest node in a road as the Neighbor_node.
    RouteModel::Node *new_neighbor =
        this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_neighbor) {
      this->neighbors.emplace_back(new_neighbor);
    }
  }
}

// Return the closest node to the input coordinate.
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  Node input;
  input.x = x;
  input.y = y;

  float min_dist = std::numeric_limits<float>::max();
  float dist;
  int closest_idx;

  for (const Model::Road &road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx : Ways()[road.way].nodes) {
        // Find the closest node with the lowest distance.
        dist = input.distance(SNodes()[node_idx]);
        if (dist < min_dist) {
          closest_idx = node_idx;
          min_dist = dist;
        }
      }
    }
  }

  return SNodes()[closest_idx];
}
#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage.
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // Find the closest nodes to the input coordinates.
  start_node = &model.FindClosestNode(start_x, start_y);
  end_node = &model.FindClosestNode(end_x, end_y);
}

// Compute the distance between the current_node to the end_node as HValue.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

// Push the vaild neibor_nodes of the current_node into the open_list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Get all of the neighbor_nodes of the current_node.
  current_node->FindNeighbors();
  for (auto &neighbor_node : current_node->neighbors) {
    // Store each path in a Tree for ConstructFinalPath().
    neighbor_node->parent = current_node;
    neighbor_node->h_value = CalculateHValue(neighbor_node);
    neighbor_node->g_value =
        current_node->g_value + current_node->distance(*neighbor_node);

    // Check Validation, operate modification, update open_list for NextNode().
    if (!neighbor_node->visited) {
      neighbor_node->visited = true;
      open_list.push_back(neighbor_node);
    }
  }
}

// Sort the open_list in descending.
void CellSort(std::vector<RouteModel::Node *> *open_list) {
  std::sort(open_list->begin(), open_list->end(),
            [](const RouteModel::Node *a, const RouteModel::Node *b) {
              return a->g_value + a->h_value > b->g_value + b->h_value;
            });
}

// Get the NextNode to search in the open_list.
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list according to the sum of the h value and g value.
  CellSort(&open_list);
  // Create a pointer to the node in the open_list with the lowest sum.
  auto node_p = open_list.back();
  // Remove that node from the open_list.
  open_list.pop_back();
  return node_p;
}

// Return the final path found from the A* search.
// The current_node is the end_node.
std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  distance = 0.0f;
  // Create path_found vector.
  std::vector<RouteModel::Node> path_found;

  // Populate the path_found.
  while (current_node && current_node != start_node) {
    // Update the path and the distance.
    path_found.push_back(*current_node);
    distance += (*current_node).parent->distance(*current_node);
    // Iterate to the next node.
    current_node = (*current_node).parent;
  }
  // Put the start_node into the path_found.
  path_found.push_back(*start_node);
  // Make path_found in the order of the start_node as the first element.
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale();
  return path_found;
}

// Create the path from the A* search.
void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node_p = nullptr;
  current_node_p = start_node;
  current_node_p->visited = true;
  // Initialize the open_list with the start_node.
  open_list.emplace_back(start_node);
 
  // A* algorithm.
  while (!open_list.empty()) { 
    auto next_p = NextNode();
    // If the search has reached the end_node, return the path. 
    if (current_node_p == end_node) { 
      m_Model.path = ConstructFinalPath(end_node);
      return;
    }
    // Update the open_list of next_node.
    AddNeighbors(next_p); 
    current_node_p = next_p;
  }
}
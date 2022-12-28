#include "route_planner.h"
#include <algorithm>

// definition of the class constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use RouteModel::Node &RouteModel::FindClosestNode(float x, float y) to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    
    /* 
        RouteModel::Node *start_node => a pointer to the start_node.
        RouteModel::Node &RouteModel::FindClosestNode(float x, float y) => a reference to the node.
        we need to point the pointer to the address, so we need & on the right hand to get the address.
    */
   start_node =  &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method. 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // HValue : the distance between the current node to the end_node 
    // Node objects have a distance method to determine the distance to another node.
    // RouteModel::Node const *node: is a pointer, needed to be dereferenced.
    return (*node).distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //  populate: add the neighbors to open_list(neighbors) of the current_node
    (*current_node).FindNeighbors();
    //  For each node in current_node.neighbors.
    for (RouteModel::Node* node : current_node->neighbors)  
    {
        if (!(*node).visited){
            // set the parent to the current_node.
            (*node).parent = current_node;
            // set the h_value to the distance between the neighbor_node and the end_node.
            (*node).h_value = CalculateHValue(node);
            // set the g_value: current_g + the distance between the current_node and the neighbor_node.!!
            (*node).g_value = current_node->g_value + current_node->distance(*node);
            //populate: add the neighbors of the neighbor_node to the open_list(neighbors) of the neighbor_node
            (*node).FindNeighbors();
            // set the node's visited attribute to true.
            (*node).visited = true;
        }
    } 
}



//Compare the F values of two nodes in the open_list[vector<node*> neighbors]
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  auto f1 = a->g_value + a->h_value;
  auto f2 = b->g_value + b->h_value;
  // Sort the open_list in descending order. 
  return f1 > f2; 
}



// only pointer can be iterative : pass by open_list as a pointer to [vector<node*>neighbors] 
void CellSort(std::vector<RouteModel::Node*> * open_list) {
    // use pointer to iterate [vector<node*>neighbors].
  std::sort(open_list->begin(), open_list->end(), Compare);
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode(RouteModel::Node *current_node) {
    // store the pointer to the neighbors of the nurrent_node in open_list.
    auto open_list = &(*current_node).neighbors;

    // Sort the open_list according to the sum of the h value and g value.
    CellSort(open_list);
    
    // Create a pointer to the node in the list with the lowest sum.
    auto node_p = (*open_list).back();

    // Remove that node from the open_list.
    (*open_list).pop_back();

    // Return the pointer.
    return node_p;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}
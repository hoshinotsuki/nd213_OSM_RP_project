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
        // set the parent to the current_node.
        (*node).parent = current_node;
        // set the h_value to the distance between the neighbor_node and the end_node.
        (*node).h_value = CalculateHValue(node);
        // set the g_value: current_g + the distance between the current_node and the neighbor_node.!!
        (*node).g_value = current_node->g_value + current_node->distance(*node); 
        if (!(*node).visited)
        {
            // adding all unvisited neighbors to the open list.
            open_list.push_back(node); 
            // set the node's visited attribute to true.
            (*node).visited = true;
        }
    } 
}
 

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
    //Compare the F values of two nodes in the open_list[vector<node*> neighbors]
    bool Compare(const RouteModel::Node* p1, const RouteModel::Node* p2) { 
    // - Sort the open_list in descending order. 
    return (p1->g_value + p1->h_value) > (p2->g_value + p2->h_value); 
    } 
    
    // only pointer can be iterative : pass by open_list as a pointer to [vector<node*>neighbors] 
    void CellSort(std::vector<RouteModel::Node*> * open_list_p) {
        // use pointer to iterate [vector<node*>neighbors].
    std::sort(open_list_p->begin(), open_list_p->end(), Compare);
    }

RouteModel::Node *RoutePlanner::NextNode() { 
    // Sort the open_list according to the sum of the h value and g value.
    CellSort(&open_list);
    // - Create a pointer to the node in the list with the lowest sum.
    auto node_p = open_list.back();
    // - Remove that node from the open_list.
    open_list.pop_back();
    // - Return the pointer.
    return node_p;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // This method should take the current (final) node as an argument 
    // iteratively follow the chain of parents of nodes until the starting node is found.
    while(current_node && current_node!=start_node)
    {
        // add node into the path
        path_found.push_back(*current_node); 
        // add the distance from the node to its parent to the distance variable.
        distance += (*current_node).parent->distance(*current_node); 
        // iterate to the next node
        current_node =(*current_node).parent;
    }
    //put the start_node into the path
    path_found.push_back(*start_node);

    // The returned vector should be in the correct order: the start node should be the first element of the vector, the end node should be the last element.
    // reverse a vector use std::reverse.
    std::reverse(path_found.begin(), path_found.end());
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here. 
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node_p = nullptr;
    // point the current_node to the start_node.
    current_node_p = start_node; 
    current_node_p->visited = true;
    // ! my mistake: don't forget to initialize the open_list with the start_node
    open_list.push_back(start_node);

    // - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
    while(open_list.size()>0)
    {
        // - Use the NextNode() method to sort the open_list and return the next node.
        auto next_p = NextNode(); 
        // - When the search has reached the end_node, 
        if (current_node_p == end_node)
        {
            // - use the ConstructFinalPath method to return the final path that was found.
            // - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }  

        // update the open_list. 
        AddNeighbors(next_p);
        // iterate to the next_node
        current_node_p = next_p;
    } 
}
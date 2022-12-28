#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

// RouteModel class inherits from the model class provided by the io2d osm example
class RouteModel : public Model {

  public:
    /* subclass(Node)inherits from the model struct(Node) .
  model struct node provided x and y coordinates. */
    class Node : public Model::Node {
      public:
    /* 
    we extend that stuct by adding :
    1. a pointer to the parent of the Node
    2. the h_value of the node
    3. the g_value of the node
    4. a boolean which tells whether the node has been visited
    5. a vector of node pointers : neighbors, which will be populated with all of the neighbors of the node
    */
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;

      // populate the neighbors 
        void FindNeighbors();
        // return the ou Distance between the current node and the other node 
        float distance(Node other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        // default constructor
        Node(){}

        /*
        constructor accepting the idx, a route model , a existing model node.
        constructor of RouteModel node from an existing model node
        index: keep tracking the node && the model to which this node belongs
        */
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

    // xml : OSM data
    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    // getter function, which returns a vector of nodes.
    auto &SNodes() { return m_Nodes; }
    // store the final path from the starting to ending 
    std::vector<Node> path;
    
  private:
    /* 
    creats a map from nodes to the roads that they belong to,
    which is useful when we want to find the neighboring node for a given node.
    */
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    // all the nodes that are in the model
    std::vector<Node> m_Nodes;

};

#endif

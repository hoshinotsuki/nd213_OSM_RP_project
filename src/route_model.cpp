#include "route_model.h"
#include <iostream>

// definition for the class constructor
// initializer list calls the model constructor with the XML data what is passed in,
// which creates the model using the XML data
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    // iterate through the nodes in that model and create RouteModel nodes for each one
    // this->Nodes() : getter function, which gets all of the nodes that are stored in model
    for (Model::Node node : this->Nodes()) {
        
        /*  for each ModelNode, we call the RouteModel node constructor,
            store those RouteModel nodes, created by the ModelNode, in the m-Nodes vector.
            counter: used as the index is passed in.
            this: a pointer to the current RouteModel being passed in.
            emplace_back: create a reference to the object that has just been pushed on to the spectrum.
            push_back: create a temporary copy
        */
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}

// !!!!hashmap:  the index of each node -> the road that it belongs to
void RouteModel::CreateNodeToRoadHashmap() {
    // iterate through all of the roads.
    // Roads() : gets all of the roads in the model.
    for (const Model::Road &road : Roads()) {
        // only includes the roads we can drive on.
        if (road.type != Model::Road::Type::Footway) {
            // Ways():  returns a vector of all the ways in the model.
            // ways()[road.way]: an index of way, to get the way the particular road belongs to.
            // way.nodes: get all of the nodes that are on that way.
            for (int node_idx : Ways()[road.way].nodes) {
                //check if each node_idx is already in the HashMap,
                // if not, we create an empty vector for that node_idx
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                // push road to the vector for the node_idx 
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

// return a pointer to the closest node in the neighbors vector to the current node.
// node_indices: current node passes those neighbors node indices.
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        // find the node_obj that belongs to that node_idx
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            // if the dis to the current node lesses than the dis to the closest_node that we've already recorded
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
            // replace the closest_node with the current node
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

// populate current_node.neighbors vector with all the neighbors.
void RouteModel::Node::FindNeighbors() {
    // node_to_road[this->index]: get the vector of the [roads] which the node belongs to, using the [current_node_index] to search the hashmap to. 
    for (auto & road : parent_model->node_to_road[this->index]) {
        // for each road, we call FindeNeighbor on the [vector of nodes] that belongs to that road.
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            // add the closest_node to the neighbors open_list
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}

// return the closest node to the input node
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            // for each of the roads, we find the way that the road belongs to with this
            /*
                struct Road {
                    enum Type { Invalid, Unclassified, Service, Residential,
                        Tertiary, Secondary, Primary, Trunk, Motorway, Footway };
                    int way;
                    Type type;
                 };
            */
            // Ways()[road.way].nodes: find the indices of all the nodes on that way.
            /*
                struct Way {
                    std::vector<int> nodes;
                };
            */
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    // update the idx and the dist
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}
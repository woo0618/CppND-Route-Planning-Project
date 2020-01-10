#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node -> distance (*end_node); 
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors) {
        if (node->visited) {
            continue;
        }
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        open_list.push_back(node);
        node->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    auto cmp = [](RouteModel::Node* node1, RouteModel::Node* node2) {
        float f1 = node1->h_value + node1->g_value;
        float f2 = node2->h_value + node2->g_value;
        return f1 == f2 ? node1->g_value > node2->g_value : f1 > f2;
    };

    std::sort(open_list.begin(), open_list.end(), cmp);
    RouteModel::Node* node = open_list.back();
    open_list.pop_back();
    return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    auto next = current_node->parent;
    while (current_node != start_node) {
        path_found.push_back(*current_node);
        distance += next->distance(*current_node);
        current_node = next;
        next = current_node->parent;
    }
    
    path_found.push_back(*start_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    start_node->visited = true;
    
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(end_node);
}

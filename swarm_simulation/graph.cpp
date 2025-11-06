#pragma once
#include "graph.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

Node::Node(double x_coord, double y_coord) : x(x_coord), y(y_coord) {}

Graph::Graph(double minDist) : minDistance(minDist) {}

Graph::~Graph() {
    for (Node* node : nodes) delete node;
}

bool Graph::shouldAddNode(double x, double y) {
    for (Node* node : nodes) {
        double dist = std::sqrt(std::pow(node->x - x, 2) + std::pow(node->y - y, 2));
        if (dist < minDistance * 0.2) { // More lenient
            return false;
        }
    }
    return true;
}

Node* Graph::addNode(Node* srcNode, double x, double y) {
    if (!shouldAddNode(x, y)) {
        // std::cout << "Node too close, skipping.\n";
        return nullptr;
    }
    Node* newNode = new Node(x, y);
    nodes.push_back(newNode);

    if (srcNode != nullptr) {
        srcNode->neighbors.push_back(newNode);
        newNode->neighbors.push_back(srcNode);  //  Add this line
    }
    return newNode;
}

void Graph::addInitialNode(Node* node) {
    nodes.push_back(node);
}

bool Graph::dfs(Node* current, Node* destination, std::vector<Node*>& path,
    std::unordered_map<Node*, bool>& visited) {
    visited[current] = true;
    path.push_back(current);

    if (current == destination)
        return true;

    for (Node* neighbor : current->neighbors) {
        if (!visited[neighbor]) {
            if (dfs(neighbor, destination, path, visited))
                return true;
        }
    }

    path.pop_back();
    return false;
}

std::vector<Node*> Graph::findPath(Node* src, Node* dest) {
    std::vector<Node*> path;
    if (!src || !dest) return path;
    std::unordered_map<Node*, bool> visited;

    if (dfs(src, dest, path, visited))
        return path;

    // std::cout << "No path found.\n";
    return {};
}

const std::vector<Node*>& Graph::getNodes() const {
    return nodes;
}

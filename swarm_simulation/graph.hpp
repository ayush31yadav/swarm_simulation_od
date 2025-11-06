#pragma once
#include <vector>
#include <unordered_map>

class Node {
public:
    double x, y;
    std::vector<Node*> neighbors;

    Node(double x_coord, double y_coord);
    ~Node() = default;
};

class Graph {
    std::vector<Node*> nodes;
    double minDistance;

    bool dfs(Node* current, Node* destination, std::vector<Node*>& path, std::unordered_map<Node*, bool>& visited);


public:
    Graph(double minDist);
    ~Graph();

    bool shouldAddNode(double x, double y);
    Node* addNode(Node* srcNode, double x, double y);
    void addInitialNode(Node* node);

    std::vector<Node*> findPath(Node* src, Node* dest);

    const std::vector<Node*>& getNodes() const;
};

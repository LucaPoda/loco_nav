#pragma once

#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include <Eigen/Dense>

struct Node {
    int id;
    Eigen::Vector2d position; 
    double score = 0.0;
};

struct Edge {
    int to;
    double weight;
};

struct EdgeDisplay {
    int u; // Node ID 1
    int v; // Node ID 2
    double weight;
};

class Roadmap {
public:
    Roadmap() = default;

    // --- Node Logic ---
    void addNode(int id, Eigen::Vector2d position, double score = 0.0);
    void addNode(int id, double x, double y, double score = 0.0);

    // --- Edge Logic ---
    void addEdge(int id1, int id2, double weight);

    // --- Getters ---
    const std::map<int, Node>& getNodes() const { return nodes_; }
    const std::vector<Edge>& getNeighbors(int id) const;
    std::vector<EdgeDisplay> getEdges() const;

    // Clear the roadmap
    void clear() {
        nodes_.clear();
        adjacency_list_.clear();
    }

private:
    std::map<int, Node> nodes_;
    std::map<int, std::vector<Edge>> adjacency_list_;
};
#ifndef ROADMAP_H
#define ROADMAP_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>

struct Node {
    int id;
    Eigen::Vector2d position; 
    double score = 0.0;
};

struct Edge {
    int to;
    double weight;
};

class Roadmap {
public:
    // --- Node Logic ---
    void addNode(int id, Eigen::Vector2d position, double score = 0.0) {
        nodes_[id] = {id, position, score};
    }

    void addNode(int id, double x, double y, double score = 0.0) {
        addNode(id, Eigen::Vector2d(x, y), score);
    }

    // --- Edge Logic ---
    void addEdge(int id1, int id2, double weight) {
        // Add edge from 1 to 2
        adjacency_list_[id1].push_back({id2, weight});
        // Add edge from 2 to 1 (making it undirected)
        adjacency_list_[id2].push_back({id1, weight});
    }

    // --- Getters ---

    const std::map<int, Node>& getNodes() const { return nodes_; }
    
    const std::vector<Edge>& getNeighbors(int id) const {
        static const std::vector<Edge> empty;
        auto it = adjacency_list_.find(id);
        return (it != adjacency_list_.end()) ? it->second : empty;
    }

private:
    std::map<int, Node> nodes_;
    std::map<int, std::vector<Edge>> adjacency_list_;
};
#endif
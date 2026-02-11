#include "environment/Roadmap.hpp" 

void Roadmap::addNode(int id, Eigen::Vector2d position, double score) {
    nodes_[id] = {id, position, score};
}

void Roadmap::addNode(int id, double x, double y, double score) {
    addNode(id, Eigen::Vector2d(x, y), score);
}

void Roadmap::addEdge(int id1, int id2, double weight) {
    adjacency_list_[id1].push_back({id2, weight});
    adjacency_list_[id2].push_back({id1, weight});
}

std::vector<Edge>& Roadmap::getNeighbors(int id) {
    static std::vector<Edge> empty;
    auto it = adjacency_list_.find(id);
    return (it != adjacency_list_.end()) ? it->second : empty;
}

std::vector<EdgeDisplay> Roadmap::getEdges() {
    std::vector<EdgeDisplay> edges;
    
    // Iterate through the adjacency map
    for (auto& entry : adjacency_list_) {
        int u = entry.first;
        std::vector<Edge>& neighbors = entry.second;

        for (auto& edge : neighbors) {
            int v = edge.to;
            // Avoid duplicate undirected edges
            if (u < v) {
                edges.push_back({u, v, edge.weight});
            }
        }
    }
    return edges;
}

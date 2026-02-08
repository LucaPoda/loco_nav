#include "orienteering/DistanceMatrix.hpp"
#include <sstream>
#include <iomanip>

DistanceMatrix::DistanceMatrix(Roadmap& original_roadmap, std::vector<int>& interest_nodes) 
    : roadmap_(original_roadmap), interest_nodes_(interest_nodes) 
{
    for (int id : interest_nodes_) {
        search_cache_[id] = computeDijkstra(id);
    }
}

DijkstraResult DistanceMatrix::computeDijkstra(int start_id) {
    DijkstraResult result;
    auto& all_nodes = roadmap_.getNodes();

    for (auto& pair : all_nodes) {
        result.distances[pair.first] = std::numeric_limits<double>::infinity();
        result.predecessors[pair.first] = -1;
    }

    result.distances[start_id] = 0.0;
    using NodeDist = std::pair<double, int>;
    std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > result.distances[u]) continue;

        for (auto& edge : roadmap_.getNeighbors(u)) {
            double new_dist = result.distances[u] + edge.weight;
            if (new_dist < result.distances[edge.to]) {
                result.distances[edge.to] = new_dist;
                result.predecessors[edge.to] = u;
                pq.push({new_dist, edge.to});
            }
        }
    }
    return result;
}

Roadmap DistanceMatrix::buildShortestPathsRoadmap() {
    Roadmap simplified_prm;
    auto& original_nodes = roadmap_.getNodes();

    // 1. Add interest nodes to the new roadmap
    for (int id : interest_nodes_) {
        auto it = original_nodes.find(id);
        if (it != original_nodes.end()) {
            simplified_prm.addNode(id, it->second.position, it->second.score);
        }
    }

    // 2. Connect them using the cached distances
    for (size_t i = 0; i < interest_nodes_.size(); ++i) {
        for (size_t j = i + 1; j < interest_nodes_.size(); ++j) {
            int u = interest_nodes_[i];
            int v = interest_nodes_[j];
            
            // Use .at() because this is a method and we know the key exists
            double dist = search_cache_.at(u).distances.at(v);
            if (dist < std::numeric_limits<double>::infinity()) {
                simplified_prm.addEdge(u, v, dist);
            }
        }
    }
    return simplified_prm;
}

std::vector<int> DistanceMatrix::getFullPath(std::vector<int>& interest_path) {
    std::vector<int> full_global_path;
    
    for (size_t i = 0; i < interest_path.size() - 1; ++i) {
        int start = interest_path[i];
        int end = interest_path[i+1];
        
        std::vector<int> segment = search_cache_.at(start).getPathTo(end);
        
        if (!full_global_path.empty() && !segment.empty()) {
            full_global_path.insert(full_global_path.end(), segment.begin() + 1, segment.end());
        } else {
            full_global_path.insert(full_global_path.end(), segment.begin(), segment.end());
        }
    }
    return full_global_path;
}

std::stringstream DistanceMatrix::display() {

    // 1. Print Header (Target IDs)
    std::stringstream ss;
    ss << "\n--- DISTANCE MATRIX ---\nID\t| ";
    for (int id : interest_nodes_) ss << id << "\t| ";
    ss << "\n-----------------------";

    // 2. Print Rows
    for (int row_id : interest_nodes_) {
        ss << "\n" << row_id << "\t| ";
        for (int col_id : interest_nodes_) {
            double dist = search_cache_.at(row_id).distances.at(col_id);
            
            if (dist >= 1e9 || dist == std::numeric_limits<double>::infinity()) {
                ss << "INF\t| ";
            } else {
                // Fixed precision for readability
                ss << std::fixed << std::setprecision(2) << dist << "\t| ";
            }
        }
    }
    
    return ss;
}
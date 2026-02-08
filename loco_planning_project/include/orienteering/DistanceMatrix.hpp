#ifndef DISTANCE_MATRIX_H
#define DISTANCE_MATRIX_H

#include "environment/Roadmap.hpp"
#include <queue>
#include <algorithm>
#include <map>

// Helper structure to store search results for a single source
struct DijkstraResult {
    std::map<int, double> distances;
    std::map<int, int> predecessors; 

    std::vector<int> getPathTo(int goal_id) {
        std::vector<int> path;
        auto it_dist = distances.find(goal_id);
        if (it_dist == distances.end() || it_dist->second == std::numeric_limits<double>::infinity()) {
            return path; 
        }
        for (int at = goal_id; at != -1; at = predecessors.at(at)) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

class DistanceMatrix {
public:
    DistanceMatrix(Roadmap& original_roadmap, std::vector<int>& interest_nodes);

    Roadmap buildShortestPathsRoadmap();

    std::vector<int> getFullPath(std::vector<int>& interest_path);

    std::stringstream display();

    // Inside DistanceMatrix class in DistanceMatrix.hpp
    std::map<int, DijkstraResult>& getSearchCache() { return search_cache_; }
    std::vector<int>& getInterestNodes() { return interest_nodes_; }

private:
    Roadmap& roadmap_;
    std::map<int, DijkstraResult> search_cache_; 
    std::vector<int> interest_nodes_;

    DijkstraResult computeDijkstra(int start_id);
};

#endif
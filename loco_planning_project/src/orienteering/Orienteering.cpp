#include "orienteering/Orienteering.hpp"

#include <algorithm>
#include <set>

std::vector<int> OrienteeringPlanner::plan(const Roadmap& simple_roadmap, int start_id, int end_id, double T_max) {
    const auto& nodes = simple_roadmap.getNodes();
    std::vector<int> current_path = {start_id, end_id};
    double current_dist = simple_roadmap.getNeighbors(start_id).at(0).weight; // Assuming direct edge exists

    // 1. GREEDY INSERTION
    std::set<int> unvisited;
    for (auto const& [id, node] : nodes) {
        if (id != start_id && id != end_id) unvisited.insert(id);
    }

    bool improved = true;
    while (improved) {
        improved = false;
        int best_node = -1;
        int best_pos = -1;
        double best_efficiency = -1.0;

        for (int candidate : unvisited) {
            for (size_t i = 0; i < current_path.size() - 1; ++i) {
                int u = current_path[i];
                int v = current_path[i+1];

                // Calculate added distance if we insert candidate between u and v
                double dist_u_c = getEdgeWeight(simple_roadmap, u, candidate);
                double dist_c_v = getEdgeWeight(simple_roadmap, candidate, v);
                double dist_u_v = getEdgeWeight(simple_roadmap, u, v);
                double added_dist = dist_u_c + dist_c_v - dist_u_v;

                if (current_dist + added_dist <= T_max) {
                    double efficiency = nodes.at(candidate).score / (added_dist + 0.001);
                    if (efficiency > best_efficiency) {
                        best_efficiency = efficiency;
                        best_node = candidate;
                        best_pos = i + 1;
                    }
                }
            }
        }

        if (best_node != -1) {
            current_path.insert(current_path.begin() + best_pos, best_node);
            unvisited.erase(best_node);
            current_dist = calculatePathDist(simple_roadmap, current_path);
            improved = true;
        }
    }

    // 2. 2-OPT LOCAL SEARCH (Optimize sequence to potentially fit more nodes)
    optimize2Opt(simple_roadmap, current_path);

    return current_path;
}

// Helper to safely get edge weight from the adjacency list
double OrienteeringPlanner::getEdgeWeight(const Roadmap& roadmap, int u, int v) {
    for (const auto& edge : roadmap.getNeighbors(u)) {
        if (edge.to == v) return edge.weight;
    }
    return 1e9; // Infinity
}

double OrienteeringPlanner::calculatePathDist(const Roadmap& roadmap, const std::vector<int>& path) {
    double total = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total += getEdgeWeight(roadmap, path[i], path[i+1]);
    }
    return total;
}

void OrienteeringPlanner::optimize2Opt(const Roadmap& roadmap, std::vector<int>& path) {
    if (path.size() < 4) return;
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 1; i < path.size() - 2; ++i) {
            for (size_t j = i + 1; j < path.size() - 1; ++j) {
                // Try reversing the segment between i and j
                double old_dist = getEdgeWeight(roadmap, path[i-1], path[i]) + getEdgeWeight(roadmap, path[j], path[j+1]);
                double new_dist = getEdgeWeight(roadmap, path[i-1], path[j]) + getEdgeWeight(roadmap, path[i], path[j+1]);
                
                if (new_dist < old_dist) {
                    std::reverse(path.begin() + i, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}
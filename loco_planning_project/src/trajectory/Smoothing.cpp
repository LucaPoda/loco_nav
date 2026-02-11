#include "trajectory/Smoothing.hpp"

std::vector<int> smoothPathVictimAware(Roadmap& roadmap, 
                                       std::vector<int>& path, 
                                       EnvironmentHandler& env,
                                       double check_resolution = 0.1) {
    if (path.size() <= 2) return path;
    auto& nodes = roadmap.getNodes();
    std::vector<int> smooth = {path[0]};

    int i = 0;
    while (i < (int)path.size() - 1) {
        int best_j = i + 1;
        
        // Try to find the furthest possible node j to shortcut to
        for (int j = i + 2; j < (int)path.size(); ++j) {
            // Victim Check: Do not skip nodes that have a score > 0
            bool victim_skipped = false;
            for (int k = i + 1; k < j; ++k) {
                if (nodes.at(path[k]).score > 0) { 
                    victim_skipped = true; 
                    break; 
                }
            }
            if (victim_skipped) break;

            // Obstacle Collision Check
            auto p_start = nodes.at(path[i]).position;
            auto p_end = nodes.at(path[j]).position;
            double dist = (p_end - p_start).norm();
            
            bool collision = false;
            // Interpolate along the straight shortcut to check for obstacles
            for (double d = 0; d < dist; d += check_resolution) {
                double t = d / dist;
                auto p_interp = p_start + t * (p_end - p_start);
                if (env.checkOccupancy(p_interp.x(), p_interp.y())) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                best_j = j; // Shortcut is valid, try the next node
            } else {
                break; // Hit an obstacle, stop searching for this node i
            }
        }
        
        smooth.push_back(path[best_j]);
        i = best_j;
    }
    return smooth;
}
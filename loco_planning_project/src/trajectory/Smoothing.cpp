// Smoothing.cpp
#include "trajectory/Smoothing.hpp"

std::vector<int> smoothPathVictimAware(const Roadmap& roadmap, const std::vector<int>& path, double max_shortcut_dist) {
    if (path.size() <= 2) return path;
    const auto& nodes = roadmap.getNodes();
    std::vector<int> smooth = {path[0]};

    int i = 0;
    while (i < (int)path.size() - 1) {
        int best_j = i + 1;
        for (int j = i + 2; j < (int)path.size(); ++j) {
            bool victim_skipped = false;
            for (int k = i + 1; k < j; ++k) {
                if (nodes.at(path[k]).score > 0) { victim_skipped = true; break; }
            }
            if (victim_skipped) break;

            double dist = (nodes.at(path[i]).position - nodes.at(path[j]).position).norm();
            if (dist < max_shortcut_dist) best_j = j;
            else break;
        }
        smooth.push_back(path[best_j]);
        i = best_j;
    }
    return smooth;
}
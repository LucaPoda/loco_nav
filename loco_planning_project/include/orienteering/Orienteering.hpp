#pragma once

#include <vector>

#include "environment/Roadmap.hpp"

class OrienteeringPlanner {
public:
    static std::vector<int> plan(const Roadmap& roadmap, int start_id, int end_id, double T_max);

private:
    static double getEdgeWeight(const Roadmap& roadmap, int u, int v);
    static double calculatePathDist(const Roadmap& roadmap, const std::vector<int>& path);
    static void optimize2Opt(const Roadmap& roadmap, std::vector<int>& path);
};

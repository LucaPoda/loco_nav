#pragma once

#include <vector>

#include "environment/Roadmap.hpp"

class OrienteeringPlanner {
public:
    static std::vector<int> plan(Roadmap& roadmap, int start_id, int end_id, double T_max);

private:
    static double getEdgeWeight(Roadmap& roadmap, int u, int v);
    static double calculatePathDist(Roadmap& roadmap, std::vector<int>& path);
    static void optimize2Opt(Roadmap& roadmap, std::vector<int>& path);
};

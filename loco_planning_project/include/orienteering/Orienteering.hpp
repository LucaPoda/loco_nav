#pragma once

#include <vector>

#include "environment/Roadmap.hpp"

class OrienteeringPlanner {
public:
    static std::vector<int> plan(const Roadmap& roadmap, int start_id, int end_id, double T_max);
};

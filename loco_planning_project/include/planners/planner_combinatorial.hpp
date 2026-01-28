#ifndef PLANNER_COMBINATORIAL_H
#define PLANNER_COMBINATORIAL_H

#include "planners/planner_base.hpp"

class PlannerCombinatorial : public PlannerBase {
public:
    PlannerCombinatorial(const std::string& robot_name);
    std::vector<Eigen::Vector3d> planPath() override;
};

#endif
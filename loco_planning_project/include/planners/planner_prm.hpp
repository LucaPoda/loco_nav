#ifndef PLANNER_PRM_H
#define PLANNER_PRM_H

#include "planners/planner_base.hpp"

class PlannerPRM : public PlannerBase {
public:
    PlannerPRM(const std::string& robot_name);
    
    // Override the virtual method
    std::vector<Eigen::Vector3d> planPath() override;

private:
    // PRM specific members (e.g., number of samples, k-nearest)
    int num_samples_;
    double connection_radius_;
};

#endif
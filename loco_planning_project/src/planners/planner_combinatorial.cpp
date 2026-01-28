#include "planners/planner_combinatorial.hpp"

PlannerCombinatorial::PlannerCombinatorial(const std::string& robot_name) 
    : PlannerBase(robot_name) 
{
    ROS_INFO("PlannerCombinatorial Initialized");
}

std::vector<Eigen::Vector3d> PlannerCombinatorial::planPath() {
    ROS_INFO("Combinatorial planning started... (Dummy Implementation)");
    return {};
}
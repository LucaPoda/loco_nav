#pragma once
#include <vector>
#include <string>
#include "environment/Roadmap.hpp"

class EnvironmentHandler;
struct TrajectoryPoint {
    double x, y, theta;
};

struct DubinsResult {
    std::vector<TrajectoryPoint> trajectory;
    bool success;           // True if we reached the final goal node
    int last_valid_idx;     // The index of the last node in 'path' we successfully reached
};

DubinsResult computeOMPLDubinsTrajectory(
    Roadmap& rm, 
    std::vector<int>& path, 
    double min_radius, 
    double step_size,
    double start_heading,
    double goal_heading,
    EnvironmentHandler& env);
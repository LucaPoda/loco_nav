#pragma once
#include <vector>
#include <string>
#include "environment/Roadmap.hpp"

struct TrajectoryPoint {
    double x, y, theta;
};

std::vector<TrajectoryPoint> computeOMPLDubinsTrajectory(const Roadmap& rm, const std::vector<int>& path, double min_radius, double step_size);
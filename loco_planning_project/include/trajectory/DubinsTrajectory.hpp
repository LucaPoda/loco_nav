#pragma once
#include <vector>
#include <string>
#include "Roadmap.hpp"

std::vector<TrajectoryPoint> computeOMPLDubinsTrajectory(const Roadmap& rm, const std::vector<int>& path, double min_radius, double step_size);
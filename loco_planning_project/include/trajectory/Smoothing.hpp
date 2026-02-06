#pragma once
#include <vector>

#include "environment/Roadmap.hpp"
#include "environment/EnvironmentHandler.hpp"

std::vector<int> smoothPathVictimAware(const Roadmap& roadmap, 
                                       const std::vector<int>& path, 
                                       const EnvironmentHandler& env,
                                       double check_resolution);
#pragma once
#include <vector>

#include "environment/Roadmap.hpp"
#include "environment/EnvironmentHandler.hpp"

std::vector<int> smoothPathVictimAware(Roadmap& roadmap, 
                                       std::vector<int>& path, 
                                       EnvironmentHandler& env,
                                       double check_resolution);
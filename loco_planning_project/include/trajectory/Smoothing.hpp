#pragma once
#include <vector>
#include "roadmap.h"

std::vector<int> smoothPathVictimAware(const Roadmap& roadmap, const std::vector<int>& path, double max_shortcut_dist);
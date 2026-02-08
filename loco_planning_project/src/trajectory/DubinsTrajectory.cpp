#include "trajectory/DubinsTrajectory.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "environment/EnvironmentHandler.hpp"

namespace ob = ompl::base;

// Standard discretisation helper 
// Helper to discretize angles.
// Center: The focus of the fan. Range: The width of the fan. K: Number of samples.
std::vector<double> getThetaSet(double center, double range, int k) {
    std::vector<double> angles;
    double step = range / (k > 1 ? k - 1 : 1);
    for (int i = 0; i < k; ++i) {
        double a = (range >= 2.0 * M_PI) ? (2.0 * M_PI * i / k) : (center - range/2.0 + i * step);
        angles.push_back(std::fmod(a + 2.0 * M_PI, 2.0 * M_PI));
    }
    return angles;
}

DubinsResult computeOMPLDubinsTrajectory(
    Roadmap& rm, std::vector<int>& path, double min_radius, 
    double step_size, double start_heading, double goal_heading, EnvironmentHandler& env) 
{
    if (path.size() < 2) return {{}, false, 0};

    auto& nodes = rm.getNodes();
    auto space = std::make_shared<ob::DubinsStateSpace>(min_radius);
    
    int n = path.size();
    int k = 24;      // Granularity
    
    // 1. Initialize Headings
    std::vector<std::vector<double>> theta_sets(n);
    theta_sets[0] = {start_heading};
    theta_sets[n-1] = {goal_heading};
    for (int i = 1; i < n - 1; ++i) theta_sets[i] = getThetaSet(0, 2.0 * M_PI, k);

    // L[j][th] = Min cost to reach Node j (at heading th) FROM START
    std::vector<std::vector<double>> L(n, std::vector<double>(k, std::numeric_limits<double>::infinity()));
    // Parent pointers to reconstruct path
    std::vector<std::vector<int>> parent_idx(n, std::vector<int>(k, -1));

    // Base Case: Start is cost 0
    L[0][0] = 0.0;
    int last_reachable_node = 0;

    // --- FORWARD PASS ---
    for (int j = 0; j < n - 1; ++j) {
        bool connection_found = false;
        auto p_curr = nodes.at(path[j]).position;
        auto p_next = nodes.at(path[j+1]).position;

        // Try connecting reachable headings at 'j' -> candidate headings at 'j+1'
        for (size_t curr_idx = 0; curr_idx < theta_sets[j].size(); ++curr_idx) {
            
            // Skip unreachable headings
            if (L[j][curr_idx] == std::numeric_limits<double>::infinity()) continue;

            for (size_t next_idx = 0; next_idx < theta_sets[j+1].size(); ++next_idx) {
                ob::ScopedState<ob::DubinsStateSpace> s1(space), s2(space);
                s1->setXY(p_curr.x(), p_curr.y()); s1->setYaw(theta_sets[j][curr_idx]);
                s2->setXY(p_next.x(), p_next.y()); s2->setYaw(theta_sets[j+1][next_idx]);

                // 1. Distance Check
                double d_len = space->distance(s1.get(), s2.get());
                
                // 2. Collision Check
                bool collision = false;
                unsigned int chk_steps = std::max(2u, (unsigned int)(d_len / 0.1));
                for (unsigned int s = 0; s <= chk_steps; ++s) {
                    ob::State *tmp = space->allocState();
                    space->interpolate(s1.get(), s2.get(), (double)s/chk_steps, tmp);
                    auto* st = tmp->as<ob::DubinsStateSpace::StateType>();
                    if (env.checkOccupancy(st->getX(), st->getY())) {
                        collision = true;
                        space->freeState(tmp); break;
                    }
                    space->freeState(tmp);
                }

                if (!collision) {
                    double new_cost = L[j][curr_idx] + d_len;
                    if (new_cost < L[j+1][next_idx]) {
                        L[j+1][next_idx] = new_cost;
                        parent_idx[j+1][next_idx] = curr_idx;
                        connection_found = true;
                    }
                }
            }
        }

        if (!connection_found) {
            // WE ARE STUCK AT NODE 'j'. CANNOT REACH 'j+1'.
            break; 
        } else {
            last_reachable_node = j + 1;
        }
    }

    // --- RECONSTRUCT PATH (Backtracking from last_reachable_node) ---
    std::vector<TrajectoryPoint> trajectory;
    
    // Find best heading at the last reachable node
    int best_th_idx = -1;
    double min_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < theta_sets[last_reachable_node].size(); ++i) {
        if (L[last_reachable_node][i] < min_cost) {
            min_cost = L[last_reachable_node][i];
            best_th_idx = i;
        }
    }

    if (best_th_idx == -1) return {{}, false, 0}; 

    // Recover indices
    std::vector<int> path_indices(last_reachable_node + 1);
    int curr_idx = best_th_idx;
    for (int j = last_reachable_node; j >= 0; --j) {
        path_indices[j] = curr_idx;
        curr_idx = parent_idx[j][curr_idx];
    }

    // Generate dense points
    for (int j = 0; j < last_reachable_node; ++j) {
        ob::ScopedState<ob::DubinsStateSpace> s_start(space), s_end(space);
        s_start->setXY(nodes.at(path[j]).position.x(), nodes.at(path[j]).position.y());
        s_start->setYaw(theta_sets[j][path_indices[j]]);
        s_end->setXY(nodes.at(path[j+1]).position.x(), nodes.at(path[j+1]).position.y());
        s_end->setYaw(theta_sets[j+1][path_indices[j+1]]);

        double d = space->distance(s_start.get(), s_end.get());
        int steps = std::max(1u, (unsigned int)(d / step_size));
        for (int s = 0; s < steps; ++s) {
            ob::State *tmp = space->allocState();
            space->interpolate(s_start.get(), s_end.get(), (double)s/steps, tmp);
            auto* ds = tmp->as<ob::DubinsStateSpace::StateType>();
            trajectory.push_back({ds->getX(), ds->getY(), ds->getYaw()});
            space->freeState(tmp);
        }
    }

    bool success = (last_reachable_node == n - 1);
    return {trajectory, success, last_reachable_node};
}
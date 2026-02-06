#include "trajectory/DubinsTrajectory.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "environment/EnvironmentHandler.hpp"

namespace ob = ompl::base;
std::vector<TrajectoryPoint> computeOMPLDubinsTrajectory(
    const Roadmap& rm, 
    const std::vector<int>& path, 
    double min_radius, 
    double step_size,
    double start_heading,
    double goal_heading,
    const EnvironmentHandler& env) 
{
    if (path.size() < 2) return {};

    const auto& nodes = rm.getNodes();
    auto space = std::make_shared<ob::DubinsStateSpace>(min_radius);
    int n = path.size();
    
    // Discretisation parameter k as defined in the complexity O(nk^2)
    int k = 36; 
    std::vector<double> theta_set;
    for (int i = 0; i < k; ++i) theta_set.push_back(2.0 * M_PI * i / k); 

    // L(j, theta_j) is the length of the solution from point Pj onwards
    std::vector<std::vector<double>> L(n, std::vector<double>(k, std::numeric_limits<double>::infinity())); 
    std::vector<std::vector<int>> best_next_theta(n, std::vector<int>(k, -1));

    // 1. Base Case: The length after the last point is 0
    // We constrain the last point to the goal_heading
    for (int i = 0; i < k; ++i) {
        if (std::abs(theta_set[i] - goal_heading) < (2.0 * M_PI / k)) { 
            L[n-1][i] = 0.0; 
        }
    }

    // 2. Backward Pass: Recursive Relation
    // L(j, theta_j) = min_{theta_j+1} (D_j(theta_j, theta_j+1) + L(j+1, theta_j+1))
    for (int j = n - 2; j >= 0; --j) {
        auto p_curr = nodes.at(path[j]).position;
        auto p_next = nodes.at(path[j+1]).position;

        for (int th_curr_idx = 0; th_curr_idx < k; ++th_curr_idx) {
            // Optimization: If j=0, we only care about headings near start_heading
            if (j == 0 && std::abs(theta_set[th_curr_idx] - start_heading) > (2.0 * M_PI / k)) continue;

            for (int th_next_idx = 0; th_next_idx < k; ++th_next_idx) {
                if (L[j+1][th_next_idx] == std::numeric_limits<double>::infinity()) continue;

                ob::ScopedState<ob::DubinsStateSpace> s1(space), s2(space);
                s1->setXY(p_curr.x(), p_curr.y()); s1->setYaw(theta_set[th_curr_idx]);
                s2->setXY(p_next.x(), p_next.y()); s2->setYaw(theta_set[th_next_idx]);

                // D_j is the length of the optimal solution of the two points problem
                double d_length = space->distance(s1.get(), s2.get()); 
                double total_len = d_length + L[j+1][th_next_idx]; 

                if (total_len < L[j][th_curr_idx]) {
                    L[j][th_curr_idx] = total_len;
                    best_next_theta[j][th_curr_idx] = th_next_idx;
                }
            }
        }
    }

    // 3. Forward Pass: Extracting the full dense trajectory
    std::vector<TrajectoryPoint> trajectory;
    
    // Find starting theta index closest to start_heading
    int current_theta_idx = -1;
    double min_diff = std::numeric_limits<double>::infinity();
    for (int i = 0; i < k; ++i) {
        double diff = std::abs(theta_set[i] - start_heading);
        if (diff < min_diff) {
            min_diff = diff;
            current_theta_idx = i;
        }
    }

    if (current_theta_idx == -1 || L[0][current_theta_idx] == std::numeric_limits<double>::infinity()) return {};

    for (int j = 0; j < n - 1; ++j) {
        int next_theta_idx = best_next_theta[j][current_theta_idx];
        if (next_theta_idx == -1) break;

        ob::ScopedState<ob::DubinsStateSpace> start(space), end(space);
        start->setXY(nodes.at(path[j]).position.x(), nodes.at(path[j]).position.y());
        start->setYaw(theta_set[current_theta_idx]);
        end->setXY(nodes.at(path[j+1]).position.x(), nodes.at(path[j+1]).position.y());
        end->setYaw(theta_set[next_theta_idx]);

        double segment_dist = space->distance(start.get(), end.get());
        unsigned int samples = std::max(1u, (unsigned int)std::ceil(segment_dist / step_size));

        for (unsigned int s = 0; s < samples; ++s) {
            ob::State *temp = space->allocState();
            space->interpolate(start.get(), end.get(), (double)s / samples, temp);
            const auto *ds = temp->as<ob::DubinsStateSpace::StateType>();
            
            // Basic collision check per interpolated point
            if (env.checkOccupancy(ds->getX(), ds->getY())) {
                space->freeState(temp);
                return {}; 
            }
            trajectory.push_back({ds->getX(), ds->getY(), ds->getYaw()});
            space->freeState(temp);
        }
        current_theta_idx = next_theta_idx;
    }
    
    trajectory.push_back({nodes.at(path.back()).position.x(), nodes.at(path.back()).position.y(), goal_heading});
    return trajectory;
}
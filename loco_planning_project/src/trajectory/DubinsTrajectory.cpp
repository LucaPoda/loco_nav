#include "trajectory/DubinsTrajectory.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <fstream>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

namespace ob = ompl::base;

std::vector<TrajectoryPoint> computeOMPLDubinsTrajectory(const Roadmap& rm, 
                                                        const std::vector<int>& path, 
                                                        double min_radius, 
                                                        double step_size) {
    if (path.empty()) return {};
    
    const auto& nodes = rm.getNodes();
    auto space = std::make_shared<ob::DubinsStateSpace>(min_radius);
    int n = path.size();
    int k = 16; // Granularity of angle discretization (k=16 from notes) [cite: 153, 258]
    
    // 1. Discretize Angles [0, 2π) [cite: 105]
    std::vector<double> theta_set;
    for (int i = 0; i < k; ++i) {
        theta_set.push_back(2.0 * M_PI * i / k);
    }

    // 2. IDP: Compute Cost-to-Go matrix L(j, theta) [cite: 126, 138]
    // L[j][theta_idx] = min length from point j (with angle theta) to the end
    std::vector<std::vector<double>> L(n, std::vector<double>(k, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<int>> best_next_theta(n, std::vector<int>(k, -1));

    // Base case: length at the last point is 0 [cite: 140]
    for (int i = 0; i < k; ++i) L[n-1][i] = 0.0;

    // Backward Pass: Compute optimal sub-problem lengths [cite: 140, 150]
    for (int j = n - 2; j >= 0; --j) {
        auto p_curr = nodes.at(path[j]).position;
        auto p_next = nodes.at(path[j+1]).position;

        for (int th_curr_idx = 0; th_curr_idx < k; ++th_curr_idx) {
            for (int th_next_idx = 0; th_next_idx < k; ++th_next_idx) {
                ob::ScopedState<ob::DubinsStateSpace> s1(space), s2(space);
                s1->setXY(p_curr.x(), p_curr.y()); s1->setYaw(theta_set[th_curr_idx]);
                s2->setXY(p_next.x(), p_next.y()); s2->setYaw(theta_set[th_next_idx]);

                double d_length = space->distance(s1.get(), s2.get());
                double total_len = d_length + L[j+1][th_next_idx];

                if (total_len < L[j][th_curr_idx]) {
                    L[j][th_curr_idx] = total_len;
                    best_next_theta[j][th_curr_idx] = th_next_idx;
                }
            }
        }
    }

    // 3. Forward Pass: Extract optimal path [cite: 114]
    std::vector<TrajectoryPoint> trajectory;
    // Find best starting angle (or fix to initial direction if needed)
    int current_theta_idx = std::min_element(L[0].begin(), L[0].end()) - L[0].begin();

    for (int j = 0; j < n - 1; ++j) {
        int next_theta_idx = best_next_theta[j][current_theta_idx];
        
        ob::ScopedState<ob::DubinsStateSpace> start(space), end(space);
        auto p1 = nodes.at(path[j]).position;
        auto p2 = nodes.at(path[j+1]).position;

        start->setXY(p1.x(), p1.y()); start->setYaw(theta_set[current_theta_idx]);
        end->setXY(p2.x(), p2.y());   end->setYaw(theta_set[next_theta_idx]);

        double segment_dist = space->distance(start.get(), end.get());
        unsigned int samples = std::max(1u, (unsigned int)std::ceil(segment_dist / step_size));

        for (unsigned int s = 0; s < samples; ++s) {
            ob::State *temp = space->allocState();
            space->interpolate(start.get(), end.get(), (double)s/samples, temp);
            const auto *ds = temp->as<ob::DubinsStateSpace::StateType>();
            trajectory.push_back({ds->getX(), ds->getY(), ds->getYaw()});
            space->freeState(temp);
        }
        current_theta_idx = next_theta_idx;
    }
    
    // Add final configuration
    auto p_fin = nodes.at(path.back()).position;
    trajectory.push_back({p_fin.x(), p_fin.y(), theta_set[current_theta_idx]});

    return trajectory;
}
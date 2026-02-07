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
std::vector<double> getThetaSet(double center, double range, int k) {
    std::vector<double> angles;
    double step = range / (k > 1 ? k - 1 : 1);
    for (int i = 0; i < k; ++i) {
        double a = (range >= 2.0 * M_PI) ? (2.0 * M_PI * i / k) : (center - range/2.0 + i * step);
        angles.push_back(std::fmod(a + 2.0 * M_PI, 2.0 * M_PI));
    }
    return angles;
}

std::vector<TrajectoryPoint> computeOMPLDubinsTrajectory(
    const Roadmap& rm, const std::vector<int>& path, double min_radius, 
    double step_size, double start_heading, double goal_heading, const EnvironmentHandler& env) 
{
    if (path.size() < 2) return {};
    const auto& nodes = rm.getNodes();
    auto space = std::make_shared<ob::DubinsStateSpace>(min_radius);
    int n = path.size();
    int k = 24;      // Granularity 
    int m_max = 4;   // Refinement steps 
    
    std::vector<std::vector<double>> theta_sets(n);
    theta_sets[0] = {start_heading};
    theta_sets[n-1] = {goal_heading};
    for (int i = 1; i < n - 1; ++i) theta_sets[i] = getThetaSet(0, 2.0 * M_PI, k);

    std::vector<int> best_indices(n, 0);
    double h = 2.0 * M_PI / k; // Initial granularity 

    for (int m = 0; m < m_max; ++m) {
        // IDP Tables 
        std::vector<std::vector<double>> L(n, std::vector<double>(k, std::numeric_limits<double>::infinity()));
        std::vector<std::vector<int>> parent_idx(n, std::vector<int>(k, -1));

        // Base Case: Final point 
        for (size_t i = 0; i < theta_sets[n-1].size(); ++i) L[n-1][i] = 0.0;

        // Backward Pass: Solve sub-problems  150]
        for (int j = n - 2; j >= 0; --j) {
            auto p_curr = nodes.at(path[j]).position;
            auto p_next = nodes.at(path[j+1]).position;

            for (size_t th_idx = 0; th_idx < theta_sets[j].size(); ++th_idx) {
                for (size_t next_idx = 0; next_idx < theta_sets[j+1].size(); ++next_idx) {
                    ob::ScopedState<ob::DubinsStateSpace> s1(space), s2(space);
                    s1->setXY(p_curr.x(), p_curr.y()); s1->setYaw(theta_sets[j][th_idx]);
                    s2->setXY(p_next.x(), p_next.y()); s2->setYaw(theta_sets[j+1][next_idx]);

                    double d_len = space->distance(s1.get(), s2.get());
                    
                    // Collision check for every candidate segment 
                    bool collision = false;
                    unsigned int chk_steps = std::max(2u, (unsigned int)(d_len / 0.1));
                    for (unsigned int s = 0; s <= chk_steps; ++s) {
                        ob::State *tmp = space->allocState();
                        space->interpolate(s1.get(), s2.get(), (double)s/chk_steps, tmp);
                        if (env.checkOccupancy(tmp->as<ob::DubinsStateSpace::StateType>()->getX(), 
                                             tmp->as<ob::DubinsStateSpace::StateType>()->getY())) {
                            collision = true;
                            space->freeState(tmp); break;
                        }
                        space->freeState(tmp);
                    }

                    if (!collision) {
                        double cost = d_len + L[j+1][next_idx];
                        if (cost < L[j][th_idx]) {
                            L[j][th_idx] = cost;
                            parent_idx[j][th_idx] = next_idx;
                        }
                    }
                }
            }
        }

        // Forward Pass: Extract solution 
        int curr = 0;
        if (L[0][curr] == std::numeric_limits<double>::infinity()) return {}; 
        for (int j = 0; j < n - 1; ++j) {
            best_indices[j] = curr;
            curr = parent_idx[j][curr];
        }

        // Refinement Step  185]
        if (m < m_max - 1) {
            h = (2.0 * M_PI) / (std::pow(3, m+1) * std::pow(2*k, m+1)); // Update h 
            for (int i = 1; i < n - 1; ++i) {
                theta_sets[i] = getThetaSet(theta_sets[i][best_indices[i]], 3.0 * h, k);
            }
        }
    }

    // Generate Final Dense Path
    std::vector<TrajectoryPoint> trajectory;
    for (int j = 0; j < n - 1; ++j) {
        ob::ScopedState<ob::DubinsStateSpace> s_start(space), s_end(space);
        s_start->setXY(nodes.at(path[j]).position.x(), nodes.at(path[j]).position.y());
        s_start->setYaw(theta_sets[j][best_indices[j]]);
        s_end->setXY(nodes.at(path[j+1]).position.x(), nodes.at(path[j+1]).position.y());
        s_end->setYaw(theta_sets[j+1][best_indices[j+1]]);

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
    return trajectory;
}
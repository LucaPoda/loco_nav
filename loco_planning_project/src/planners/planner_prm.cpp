#include "planners/planner_prm.hpp"
#include <pluginlib/class_list_macros.h>
#include <random>
#include <algorithm>
#include <queue>
#include <map>

PlannerPRM::PlannerPRM() {}
PlannerPRM::~PlannerPRM() {}

void PlannerPRM::initialize(const std::string& robot_name) {
    PlannerBase::initialize(robot_name);

    // Load PRM-specific params
    ros::NodeHandle pnh("~");
    pnh.param<int>("prm/n_samples", n_samples, 200);
    pnh.param<int>("prm/k_neighbors", k_neighbors, 10);
    pnh.param<double>("prm/resolution", resolution, 0.05);
    pnh.param<double>("prm/min_connection_distance", params_.min_connection_distance, 4.0 * params_.min_radius);
    pnh.param<double>("prm/max_connection_distance", params_.max_connection_distance, 4.0);


    ROS_INFO("PRM initialized: %d samples, k=%d, res=%.3f", n_samples, k_neighbors, resolution);
}

Roadmap PlannerPRM::buildRoadmap() {
    ROS_INFO("Initializing map parameters...");
    Roadmap roadmap;

    const auto& env = getEnvironment();

    std::vector<Eigen::Vector2d> node_positions;

    double min_x = env.getMinX();
    double max_x = env.getMaxX();
    double min_y = env.getMinY();
    double max_y = env.getMaxY();

    // Add special nodes
    Eigen::Vector2d start_pos = env.getStartPose().head<2>();
    Eigen::Vector2d goal_pos = env.getGoalPose().head<2>();
    const auto& victims = env.getVictims();

    // Define IDs
    int start_id = 0;
    int goal_id  = 1;
    int victim_start_index = 2;

    // Add to roadmap and to positions vector
    ROS_INFO("Adding special nodes...");

    // Start
    roadmap.addNode(start_id, start_pos, 0.0);
    node_positions.push_back(start_pos);

    // Goal
    roadmap.addNode(goal_id, goal_pos, 0.0);
    node_positions.push_back(goal_pos);

    // Victims
    for (size_t i = 0; i < victims.size(); ++i) {
        int v_id = victim_start_index + i;
        roadmap.addNode(v_id, victims[i].position, victims[i].reward); 
        node_positions.push_back(victims[i].position);
    }

    
    // Random Sampling
    ROS_INFO("PRM: Sampling %d nodes...", n_samples); 

    std::uniform_real_distribution<double> dist_x(min_x, max_x);
    std::uniform_real_distribution<double> dist_y(min_y, max_y);
    std::default_random_engine generator;

    while (node_positions.size() < static_cast<size_t>(n_samples)) {
        double rx = dist_x(generator);
        double ry = dist_y(generator);

        // Check if the point is inside an obstacle
        if (!env.checkOccupancy(rx, ry)) {
            Eigen::Vector2d pos(rx, ry);
            int current_id = static_cast<int>(node_positions.size());
            
            roadmap.addNode(current_id, pos, 0.0); 
            node_positions.push_back(pos);
        }
    }

    // Connect neighbours
    ROS_INFO("Connetting nodes...");
    ROS_INFO("Found %zu positions...", node_positions.size());
    for (int i = 0; i < (int)node_positions.size(); ++i) {
        std::vector<std::pair<double, int>> neighbors;

        for (int j = 0; j < (int)node_positions.size(); ++j) {
            if (i == j) continue;

            double d = (node_positions[i] - node_positions[j]).norm();

            // If distance is less than 4R, we skip this neighbor to avoid CCC/Lightbulb loops
            if (d < params_.min_connection_distance) {
                continue;
            }

            // upper bound to keep the graph sparse
            if (d > params_.max_connection_distance) {
                continue;
            }

            neighbors.push_back({d, j});
        }

        // Sort by distance to get the "K" closest valid candidates
        std::sort(neighbors.begin(), neighbors.end());

        int connections_made = 0;
        for (size_t k = 0; k < neighbors.size() && connections_made < k_neighbors; ++k) {
            int neighbor_idx = neighbors[k].second;
            double distance = neighbors[k].first;

            if (isCollisionFree(node_positions[i], node_positions[neighbor_idx])) {
                roadmap.addEdge(i, neighbor_idx, distance);
                connections_made++;
            }
        }
    }

    // Return the roadmap
    ROS_INFO("Roadmap ready.");
    return roadmap;
}

bool PlannerPRM::isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    const auto& env = getEnvironment();
    
    // Safety check
    double res = (resolution <= 0.0) ? 0.05 : resolution;
    
    double dist = (p1 - p2).norm();
    if (dist < 0.001) return true;

    // Line interpolation
    int steps = std::max(1, static_cast<int>(dist / res));    
    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / static_cast<double>(steps);
        Eigen::Vector2d interpolated = p1 + ratio * (p2 - p1);

        if (env.checkOccupancy(interpolated.x(), interpolated.y())) {
            return false;
        }
    }
    return true;
}

PLUGINLIB_EXPORT_CLASS(PlannerPRM, PlannerBase)
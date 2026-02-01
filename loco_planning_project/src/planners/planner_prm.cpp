#include "planners/planner_prm.hpp"
#include <pluginlib/class_list_macros.h>
#include <random>    // Required for std::default_random_engine
#include <algorithm> // Required for std::sort and std::min

PlannerPRM::PlannerPRM() {}
PlannerPRM::~PlannerPRM() {}

void PlannerPRM::initialize(const std::string& robot_name) {
    // 1. Initialize Base (Sets up env_ and nh_)
    PlannerBase::initialize(robot_name);

    // 2. Load PRM-specific params
    ros::NodeHandle pnh("~");
    pnh.param<int>("prm/n_samples", n_samples, 200);
    pnh.param<int>("prm/k_neighbors", k_neighbors, 10);
    pnh.param<double>("prm/resolution", resolution, 0.05);

    ROS_INFO("PRM initialized: %d samples, k=%d, res=%.3f", n_samples, k_neighbors, resolution);
}

Roadmap PlannerPRM::buildRoadmap() {
    ROS_INFO("Initializing map parameters...");
    Roadmap roadmap;
    // std::vector<geometry_msgs::Point> nodes;

    // Use the protected getter from PlannerBase
    const auto& env = getEnvironment();
    // double width = env.getMapWidth();
    // double height = env.getMapHeight();

    // We keep a local vector of positions to make distance checking easy
    std::vector<Eigen::Vector2d> node_positions;

    
    // 1. Random Sampling
    ROS_INFO("PRM: Sampling %d nodes...", n_samples); 
    
    // Use the real boundaries
    double min_x = env.getMapBorders().getMinX();
    double max_x = env.getMapBorders().getMaxX();
    double min_y = env.getMapBorders().getMinY();
    double max_y = env.getMapBorders().getMaxY();

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
            
            // Matches your Roadmap class: addNode(int id, Eigen::Vector2d position, double score)
            roadmap.addNode(current_id, pos, 0.0); 
            node_positions.push_back(pos);
        }
    }

    // 2. Connect Neighbors
    ROS_INFO("Connetting nodes...");
    ROS_INFO("Found %zu positions...", node_positions.size());
    for (int i = 0; i < node_positions.size(); ++i) {
        std::vector<std::pair<double, int>> neighbors;

        for (int j = 0; j < node_positions.size(); ++j) {
            if (i == j) continue;

            // Use node_positions.size()
            if (i >= node_positions.size() || j >= node_positions.size()) {
                continue;
        }

        double d = (node_positions[i] - node_positions[j]).norm();
        neighbors.push_back({d, j});
    }
        // Sort by distance to get the "K" closest
        std::sort(neighbors.begin(), neighbors.end());

        int connections_made = 0;
        for (size_t k = 0; k < neighbors.size() && connections_made < k_neighbors; ++k) {
            int neighbor_idx = neighbors[k].second;
            double distance = neighbors[k].first;

            ROS_INFO("Checking collision between %d and %d", i, neighbor_idx);
            // Check if the straight-line path between i and neighbor_idx is clear
            if (isCollisionFree(node_positions[i], node_positions[neighbor_idx])) {
                roadmap.addEdge(i, neighbor_idx, distance);
                connections_made++;
            }
        }
    }

    ROS_INFO("Map ready.");
    return roadmap;
}

bool PlannerPRM::isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    const auto& env = getEnvironment();
    ROS_INFO("Checking points");
    
    // SAFETY CHECK: Ensure resolution is valid
    if(resolution <= 0.0){
        resolution = 0.05;
        ROS_INFO("Resolution set was not valid. Setting resolution to 0.05.");
    }
    
    double dist = (p1 - p2).norm();
    if (dist < 0.001) return true;

    int steps = std::max(1, static_cast<int>(dist / resolution));    
    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / static_cast<double>(steps);
        double x = p1.x() + (p2.x() - p1.x()) * ratio;
        double y = p1.y() + (p2.y() - p1.y()) * ratio;

        // --- ADD BOUNDS CHECKING HERE ---
        if (x < 0 || x >= env.getMapWidth() || y < 0 || y >= env.getMapHeight()) {
            return false; // Treat out-of-bounds as a collision
        }

        // Only call the env function if we are safely inside the map
        if (env.checkOccupancy(x, y)) {
            return false;
        }
    }
    return true;
}

PLUGINLIB_EXPORT_CLASS(PlannerPRM, PlannerBase)
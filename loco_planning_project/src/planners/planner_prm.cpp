#include "planners/planner_prm.hpp"
#include <pluginlib/class_list_macros.h>
#include <random>    // Required for std::default_random_engine
#include <algorithm> // Required for std::sort and std::min
#include <queue>
#include <map>

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
    this->special_ids.clear();
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
    double min_x = env.getMinX();
    double max_x = env.getMaxX();
    double min_y = env.getMinY();
    double max_y = env.getMaxY();

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

    // 2. Add special nodes
    // 2.1 Get special nodes: start, goal and victims
    Eigen::Vector2d start_pos = env.getStartPose().head<2>();
    Eigen::Vector2d goal_pos = env.getGoalPose().head<2>();
    const auto& victims = env.getVictims();

    // 2.2 Define IDs (high to avoid overlap with random nodes' IDs)
    int start_id = n_samples;
    int goal_id  = n_samples + 1;
    int victim_start_index = n_samples + 2; // first victim, start victims ID from here

    // 2.3 Add to roadmap and to positions vector
    ROS_INFO("Adding special nodes...");

    // Start
    roadmap.addNode(start_id, start_pos, 0.0); // Start has 0 score
    node_positions.push_back(start_pos);
    special_ids.push_back(start_id);

    // Goal
    roadmap.addNode(goal_id, goal_pos, 0.0);   // Goal has 0 score
    node_positions.push_back(goal_pos);
    special_ids.push_back(goal_id);

    // Victims
    for (size_t i = 0; i < victims.size(); ++i) {
        int v_id = victim_start_index + i;
        // IMPORTANT: Score comes from the 'radius' field of the topic
        roadmap.addNode(v_id, victims[i].position, victims[i].reward); 
        node_positions.push_back(victims[i].position);
        special_ids.push_back(victim_start_index + i);
    }

    // 3. Connect neighbours
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

    // 4. Return the roadmap
    ROS_INFO("Roadmap ready.");
    return roadmap;
}

bool PlannerPRM::isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    const auto& env = getEnvironment();
    
    // 1. Safety check for resolution
    double res = (resolution <= 0.0) ? 0.05 : resolution;
    
    double dist = (p1 - p2).norm();
    if (dist < 0.001) return true;

    // 2. Line interpolation
    int steps = std::max(1, static_cast<int>(dist / res));    
    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / static_cast<double>(steps);
        Eigen::Vector2d interpolated = p1 + ratio * (p2 - p1);

        // REMOVED: if (x < 0 || x >= width...) 
        // WHY: Your hexagon has negative coordinates. env.checkOccupancy
        // already knows how to handle boundaries correctly!
        
        if (env.checkOccupancy(interpolated.x(), interpolated.y())) {
            return false;
        }
    }
    return true;
}

// Dijkstra using priority queue
// 1. SIMPLE DIJKSTRA: Just returns a map of distances from 'start_id' to everyone else
std::map<int, double> PlannerPRM::dijkstraDistances(int start_id, const Roadmap& roadmap) {
    std::map<int, double> distances;
    
    // Initialize all distances to infinity
    for (const auto& node_pair : roadmap.getNodes()) {
        distances[node_pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[start_id] = 0.0;

    // Priority queue to always expand the closest node
    using NodeDist = std::pair<double, int>;
    std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // If we found a longer path than we already have, skip it
        if (d > distances[u]) continue;

        for (const auto& edge : roadmap.getNeighbors(u)) {
            if (distances[u] + edge.weight < distances[edge.to]) {
                distances[edge.to] = distances[u] + edge.weight;
                pq.push({distances[edge.to], edge.to});
            }
        }
    }
    return distances;
}

// 2. MATRIX COMPUTATION: Fills the NxN table
std::map<int, std::map<int, double>> PlannerPRM::computeSpecialNodesMatrix(const Roadmap& roadmap) {
    std::map<int, std::map<int, double>> cost_matrix;

    // 1. Compute the matrix
    for (int source : special_ids) {
        std::map<int, double> results = dijkstraDistances(source, roadmap);
        for (int target : special_ids) {
            cost_matrix[source][target] = results[target];
        }
    }

    // 2. Print Header (Target IDs)
    std::stringstream ss;
    ss << "\n--- DISTANCE MATRIX ---\nID\t| ";
    for (int id : special_ids) ss << id << "\t| ";
    ss << "\n-----------------------";

    // 3. Print Rows
    for (int row_id : special_ids) {
        ss << "\n" << row_id << "\t| ";
        for (int col_id : special_ids) {
            double dist = cost_matrix[row_id][col_id];
            
            if (dist >= 1e9 || dist == std::numeric_limits<double>::infinity()) {
                ss << "INF\t| ";
            } else {
                // Fixed precision for readability
                ss << std::fixed << std::setprecision(2) << dist << "\t| ";
            }
        }
    }
    
    ROS_INFO_STREAM(ss.str() << "\n-----------------------");
    return cost_matrix;
}

PLUGINLIB_EXPORT_CLASS(PlannerPRM, PlannerBase)
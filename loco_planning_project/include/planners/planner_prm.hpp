#ifndef PLANNER_PRM_H
#define PLANNER_PRM_H

#include "planners/planner_base.hpp"
#include <ros/ros.h>

class PlannerPRM : public PlannerBase {
public:
    // 1. Default Constructor (Required by pluginlib)
    PlannerPRM();

    // 2. Destructor
    virtual ~PlannerPRM();

    // 3. Initialization Override
    // We override this to load PRM-specific parameters
    void initialize(const std::string& robot_name) override;

protected:
    // 4. The Core Logic Override
    Roadmap buildRoadmap() override;

private:
    // PRM-specific parameters and helepers here
    // Parameters
    int n_samples;
    int k_neighbors;
    double resolution;

    std::vector<int> special_ids;

    // Helpers
    // Using Eigen for internal planning logic is significantly more efficient than geometry_msgs::Point
    bool isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);     

    // Dijkstra algorithm for shortest path on the graph
    std::map<int, double> dijkstraDistances(int start_id, const Roadmap& roadmap);

    // NxN distance matrix for special nodes
    std::map<int, std::map<int, double>> computeSpecialNodesMatrix(const Roadmap& roadmap) override;
};
#endif


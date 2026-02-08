#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <loco_planning/Reference.h>

#include "environment/EnvironmentHandler.hpp"
#include "environment/Roadmap.hpp"
#include "orienteering/DistanceMatrix.hpp"
#include "orienteering/Orienteering.hpp"
#include "trajectory/Smoothing.hpp"
#include "trajectory/DubinsTrajectory.hpp"
#include "utils/PlannerVisualizer.hpp"

struct PlannerParams {
    double v_max;
    double curvature_max;
    double dt;

    double t_max;
    double min_radius;
    double step_size;
    double min_connection_distance; 
    double max_connection_distance; 
    PlannerParams() : v_max(0.1), curvature_max(1.0), dt(0.01), t_max(400), step_size(0.05), min_connection_distance(1.6), max_connection_distance(4.0) {}
};

class PlannerBase {
public:
    PlannerBase() : path_computed_(false) {}
    virtual ~PlannerBase() {}

    // 1. Initialization 
    virtual void initialize(const std::string& robot_name);

    // 2. The Main Loop
    void run();

protected:
    const EnvironmentHandler& getEnvironment() const { 
        return env_; 
    }

    const PlannerParams& getParams() const { 
        return params_; 
    }

    // Abstract method to calculate the geometric path
    virtual Roadmap buildRoadmap() = 0;

    // Compute matrix with dijkstra
    // virtual std::map<int, std::map<int, double>> computeSpecialNodesMatrix(const Roadmap& roadmap) = 0;   
    
    ros::Publisher roadmap_pub_;
    // Generic function to visualize any roadmap graph
    //void visualizeRoadmap(const Roadmap& roadmap);

    PlannerVisualizer visualizer_;

    // Environment data form ROS:
    EnvironmentHandler env_;
    // Planner parameters: v_max, curvature_max, dt
    PlannerParams params_; 


private:
    // Node Handle for global topics
    ros::NodeHandle nh_;

    // Node Handle for private params (~/param)
    ros::Publisher ref_pub_;
    
    std::string robot_name_;
    bool path_computed_;

    double max_path_length_;

    // Converts geometric waypoints into a dense, timed trajectory.
    std::vector<loco_planning::Reference> computeReferenceFromPath(const std::vector<TrajectoryPoint>& path);
    
    // Publishes the trajectory to the reference topic.
    void publishReference(const std::vector<loco_planning::Reference>& reference);

    std::vector<int> special_ids; // [0, n_victims+2)
};

#endif // PLANNER_BASE_H
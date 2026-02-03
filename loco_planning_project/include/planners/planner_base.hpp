#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <ros/ros.h>
#include <loco_planning/Reference.h>
#include <Eigen/Dense>
#include "utils/environment_handler.hpp"
#include "utils/roadmap.hpp"
#include <vector>
#include <string>
#include "utils/planner_visualizer.hpp"

struct PlannerParams {
    double v_max;
    double curvature_max;
    double dt;
    PlannerParams() : v_max(0.1), curvature_max(1.0), dt(0.01) {}
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
    virtual std::map<int, std::map<int, double>> computeSpecialNodesMatrix(const Roadmap& roadmap) = 0;   
    
    ros::Publisher roadmap_pub_;
    // Generic function to visualize any roadmap graph
    //void visualizeRoadmap(const Roadmap& roadmap);

    PlannerVisualizer visualizer_;

private:
    // Environment data form ROS:
    EnvironmentHandler env_;
    // Planner parameters: v_max, curvature_max, dt
    PlannerParams params_; 

    
    // Node Handle for global topics
    ros::NodeHandle nh_;

    // Node Handle for private params (~/param)
    ros::Publisher ref_pub_;
    
    std::string robot_name_;
    bool path_computed_;

    // Converts geometric waypoints into a dense, timed trajectory.
    std::vector<loco_planning::Reference> computeReferenceFromPath(const std::vector<Eigen::Vector3d>& path);
    
    // Publishes the trajectory to the reference topic.
    void publishReference(const std::vector<loco_planning::Reference>& reference);
};

#endif // PLANNER_BASE_H
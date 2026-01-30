#include "planners/planner_prm.hpp"
#include <pluginlib/class_list_macros.h> // CRITICAL: Registers the class

// 1. Default Constructor
PlannerPRM::PlannerPRM() {
    // Minimal setup. Real work happens in initialize().
}

PlannerPRM::~PlannerPRM() {}

// 2. Initialization
void PlannerPRM::initialize(const std::string& robot_name) {
    // A. ALWAYS call the Base initialization first!
    // This sets up the EnvironmentHandler, publishers, and global params.
    PlannerBase::initialize(robot_name);

    ROS_INFO("Initializing PRM Planner plugin...");

    // B. Load Algorithm-Specific Parameters
    ros::NodeHandle pnh("~");
    
    // Example: Load 'prm/<param_name>' from yaml
    // Using a default of 1.0 if not found
    // pnh.param<double>("prm/<param_name>", heuristic_weight_, 1.0);

    ROS_INFO("PRM Config -- <params>");
}

// 3. The Planning Logic
Roadmap PlannerPRM::buildRoadmap() {
    Roadmap roadmap;

    // TODO: build roadmap (graph) with PRM 

    return roadmap;
}

// 4. Register the Plugin
// Arguments: (Derived Class, Base Class)
PLUGINLIB_EXPORT_CLASS(PlannerPRM, PlannerBase)
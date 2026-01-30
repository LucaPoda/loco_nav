#include "planners/planner_ecd.hpp"
#include <pluginlib/class_list_macros.h> // CRITICAL: Registers the class

// 1. Default Constructor
PlannerECD::PlannerECD() {
    // Minimal setup. Real work happens in initialize().
}

PlannerECD::~PlannerECD() {}

// 2. Initialization
void PlannerECD::initialize(const std::string& robot_name) {
    // A. ALWAYS call the Base initialization first!
    // This sets up the EnvironmentHandler, publishers, and global params.
    PlannerBase::initialize(robot_name);

    ROS_INFO("Initializing ECD Planner plugin...");

    // B. Load Algorithm-Specific Parameters
    ros::NodeHandle pnh("~");
    
    // Example: Load 'ecd/<param>' from yaml
    // Using a default of 1.0 if not found
    // pnh.param<double>("ecd/<param>", <param>_, 1.0);

    ROS_INFO("ECD Config -- <params>"   );
}

// 3. The Planning Logic
Roadmap PlannerECD::buildRoadmap() {
    Roadmap roadmap;

    // TODO: build roadmap (graph) with Exact Cell Decomposition

    return roadmap;
}

// 4. Register the Plugin
// Arguments: (Derived Class, Base Class)
PLUGINLIB_EXPORT_CLASS(PlannerECD, PlannerBase)
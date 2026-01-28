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
std::vector<Eigen::Vector3d> PlannerECD::planPath() {
    // A. ACCESS DATA (Read-Only)
    // Use the protected getters from PlannerBase
    const auto& obstacles = getEnvironment().getObstacles();
    const auto& start = getEnvironment().getStartPose();
    const auto& goal = getEnvironment().getGoalPose();
    
    // Check constraints (e.g. from planning.yaml)
    // double v_max = getParams().v_max;

    ROS_INFO("ECD: Planning from [%.2f, %.2f] to [%.2f, %.2f]",
             start.x(), start.y(), goal.x(), goal.y());

    std::vector<Eigen::Vector3d> path;

    // --- YOUR ALGORITHM GOES HERE ---
    
    // TODO: Implement the ECD algorithm

    // Example dummy path (Start -> Goal) just to test the pipeline
    if (path.empty()) {
        path.push_back(start);
        path.push_back(goal);
    }

    return path;
}

// 4. Register the Plugin
// Arguments: (Derived Class, Base Class)
PLUGINLIB_EXPORT_CLASS(PlannerECD, PlannerBase)
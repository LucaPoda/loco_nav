#include <ros/ros.h>
#include "planners/planner_prm.hpp"
#include "planners/planner_combinatorial.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    // Read parameters
    std::string robot_name;
    std::string planning_method;
    
    nh.param<std::string>("robot_name", robot_name, "limo0");
    nh.param<std::string>("planning_method", planning_method, "prm");

    ROS_INFO("Starting Planner Node for %s using method: %s", robot_name.c_str(), planning_method.c_str());

    // Polymorphism: Pointer to Base, pointing to Derived
    std::unique_ptr<PlannerBase> planner;

    if (planning_method == "prm") {
        planner = std::make_unique<PlannerPRM>(robot_name);
    } 
    else if (planning_method == "combinatorial") {
        planner = std::make_unique<PlannerCombinatorial>(robot_name);
    } 
    else {
        ROS_ERROR("Unknown planning method: %s. Using PRM default.", planning_method.c_str());
        planner = std::make_unique<PlannerPRM>(robot_name);
    }

    // Initialize the chosen planner
    planner->init();

    // Start the planning loop
    planner->run();

    return 0;
}
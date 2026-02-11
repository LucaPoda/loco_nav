#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "planners/planner_base.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    std::string robot_name;
    std::string planning_plugin;
    
    nh.param<std::string>("robot_name", robot_name, "limo0");
    
    // Default to PRM if not specified
    nh.param<std::string>("planning_plugin", planning_plugin, "loco_planning_project/PlannerPRM");

    ROS_INFO("Starting Planner Node for %s", robot_name.c_str());

    // Setup plugin loader
    pluginlib::ClassLoader<PlannerBase> loader("loco_planning_project", "PlannerBase"); 

    try {
        ROS_INFO("Loading planner plugin: %s...", planning_plugin.c_str());

        // Load plugin and initialize
        boost::shared_ptr<PlannerBase> planner = loader.createInstance(planning_plugin);
        planner->initialize(robot_name);

        ROS_INFO("Plugin loaded successfully. Starting main loop...");

        // start
        planner->run();

    } catch(pluginlib::PluginlibException& ex) {
        // Handle errors like: typo in the name, library not found, etc.
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    return 0;
}
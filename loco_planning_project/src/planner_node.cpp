#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "planners/planner_base.hpp" // Only needs the Base Class header!

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    // 1. READ PARAMETERS
    std::string robot_name;
    std::string planning_plugin; // This replaces "planning_method"
    
    nh.param<std::string>("robot_name", robot_name, "limo0");
    
    // Default to PRM if not specified
    // "loco_planning_project" is the package name (matches your XML)
    // "PlannerPRM" is the class name (matches your XML)
    nh.param<std::string>("planning_plugin", planning_plugin, "loco_planning_project/PlannerPRM");

    ROS_INFO("Starting Planner Node for %s", robot_name.c_str());

    // 2. SETUP PLUGIN LOADER
    // Arguments: (Package Name of the Base Class, Fully Qualified Base Class Type)
    // Note: Make sure "loco_planning_project" matches the package name in your package.xml
    pluginlib::ClassLoader<PlannerBase> loader("loco_planning_project", "PlannerBase"); 

    try {
        ROS_INFO("Loading planner plugin: %s...", planning_plugin.c_str());

        // 3. LOAD THE PLUGIN
        // This effectively does: planner = new PlannerPRM();
        // createInstance returns a boost::shared_ptr
        boost::shared_ptr<PlannerBase> planner = loader.createInstance(planning_plugin);

        // 4. INITIALIZE
        // We must call this manually because pluginlib constructors cannot take arguments
        planner->initialize(robot_name);

        ROS_INFO("Plugin loaded successfully. Starting main loop...");

        // 5. RUN
        planner->run();

    } catch(pluginlib::PluginlibException& ex) {
        // This handles errors like: typo in the name, library not found, etc.
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    return 0;
}
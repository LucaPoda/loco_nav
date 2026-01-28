#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <loco_planning/Reference.h> // Your custom message
#include <Eigen/Dense>

#include "utils/obstacles.hpp"
#include "utils/planner_visualizer.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// Parameter structure
struct PlannerParams {
    double robot_radius;
    double v_max;
    double curvature_max;
    double dt;

    PlannerParams() : robot_radius(0.2), v_max(0.1), curvature_max(1.0), dt(0.01) {}
};

class PlannerBase {
public:
    PlannerBase(const std::string& robot_name);
    ~PlannerBase();

    // Initialization routine
    void init();

    // Main blocking loop (replaces rospy.spin inside main)
    void run();

protected:
    // ---------------------------------------------------------
    // VIRTUAL METHODS (To be implemented by Subclasses like PRM/(todo: decide combinatorial alg,))
    // ---------------------------------------------------------
    
    /**
     * @brief Abstract method to calculate the geometric path.
     * @return A vector of 3D points (x, y, theta). Theta is optional depending on planner.
     */
    virtual std::vector<Eigen::Vector3d> planPath() = 0;


    // ---------------------------------------------------------
    // COMMON LOGIC (Available to all planners)
    // ---------------------------------------------------------

    /**
     * @brief Converts a geometric path (waypoints) into a dense, timed trajectory.
     * Handles Dubins smoothing or Piecewise linear interpolation.
     */
    virtual std::vector<loco_planning::Reference> computeReferenceFromPath(const std::vector<Eigen::Vector3d>& path);

    /**
     * @brief Publishes the trajectory to the reference topic.
     */ 
    void publishReference(const std::vector<loco_planning::Reference>& reference);

    // Callbacks
    void mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg);
    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Helpers
    std::vector<Obstacle> discretizeBorder(const geometry_msgs::Polygon& poly, int discretization_points, double radius);

    // ---------------------------------------------------------
    // MEMBER VARIABLES
    // ---------------------------------------------------------
    ros::NodeHandle nh_;
    ros::Publisher ref_pub_;
    ros::Subscriber sub_borders_, sub_obs_, sub_goal_, sub_odom_;

    std::string robot_name_;
    PlannerParams params_;

    // State flags
    bool map_ready_;
    bool obstacles_ready_;
    bool goal_ready_;
    bool start_ready_;
    bool path_computed_;

    // Planning Data
    double safety_margin_;
    
    Eigen::Vector3d start_pose_; // x, y, theta
    Eigen::Vector3d goal_pose_;  // x, y, theta
    std::vector<Obstacle> obstacles_; // Stores dynamic obstacles
    Obstacle border_obstacle_; // Stores dynamic obstacles

    PlannerVisualizer visualizer_;
};

#endif // PLANNER_BASE_H
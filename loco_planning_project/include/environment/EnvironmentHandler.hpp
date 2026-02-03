#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "environment/Obstacle.hpp"
#include "utils/PlannerVisualizer.hpp"

struct VictimData {
    Eigen::Vector2d position;
    double reward;
};

class EnvironmentHandler {
public:
    EnvironmentHandler();

    void init(ros::NodeHandle& nh, const std::string& robot_name);

    // --- GETTERS ---
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    const Obstacle& getMapBorders() const { return border_obstacle_; } 
    const Eigen::Vector3d& getStartPose() const { return start_pose_; }
    const Eigen::Vector3d& getGoalPose() const { return goal_pose_; }
    const std::vector<VictimData>& getVictims() const { return victims_; }    
    
    bool isReady() const;
    bool checkOccupancy(double x, double y) const;
    double getMapWidth() const;
    double getMapHeight() const;
    double getMinX() const { return border_obstacle_.getMinX(); }
    double getMaxX() const { return border_obstacle_.getMaxX(); }
    double getMinY() const { return border_obstacle_.getMinY(); }
    double getMaxY() const { return border_obstacle_.getMaxY(); }

private:
    // --- CALLBACKS ---
    void mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg);
    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg);

    // --- DATA ---
    double robot_radius_;
    double safety_margin_;
    double inflation_epsilon_;
    double total_inflation_;

    std::vector<Obstacle> obstacles_;
    Obstacle border_obstacle_;
    Eigen::Vector3d start_pose_;
    Eigen::Vector3d goal_pose_;
    std::vector<VictimData> victims_;

    bool map_borders_ready_;
    bool obstacles_ready_;
    bool goal_ready_;
    bool start_ready_;
    bool victims_ready_;

    ros::Subscriber sub_borders_, sub_obs_, sub_goal_, sub_odom_, sub_victims_;
    PlannerVisualizer visualizer_; 
};

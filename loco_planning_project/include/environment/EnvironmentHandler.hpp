#ifndef ENVIRONMENT_HANDLER_H
#define ENVIRONMENT_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h> // Added for map borders
#include <nav_msgs/Odometry.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>

// TF2 Includes (Required for Quaternion -> Yaw conversion)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "environemnt/Obstacles.hpp"
#include "utils/PlannerVisualizer.hpp" // Include your visualizer

struct VictimData {
    Eigen::Vector2d position;
    double reward;
};

class EnvironmentHandler {
public:
    EnvironmentHandler() 
        : map_borders_ready_(false), 
          obstacles_ready_(false), 
          goal_ready_(false), 
          start_ready_(false),
          victims_ready_(false) 
    { 
        // Default constructor
    }

    void init(ros::NodeHandle& nh, const std::string& robot_name) {
        ros::NodeHandle pnh("~"); 

        // 1. Load Parameters
        pnh.param<double>("robot_radius", robot_radius_, 0.2);
        pnh.param<double>("safety_margin", safety_margin_, 0.05);
        pnh.param<double>("inflation_epsilon", inflation_epsilon_, 0.01);

        // Pre-calculate total inflation
        total_inflation_ = robot_radius_ + safety_margin_;

        ROS_INFO("EnvHandler: Params loaded. Radius: %.2f, Margin: %.2f -> Total Inflation: %.3f", 
                robot_radius_, safety_margin_, total_inflation_);

        // 2. Setup Subscribers
        sub_borders_ = nh.subscribe("/map_borders", 1, &EnvironmentHandler::mapBordersCallback, this);
        sub_obs_ = nh.subscribe("/obstacles", 1, &EnvironmentHandler::obstaclesCallback, this);
        sub_goal_ = nh.subscribe("/gates", 1, &EnvironmentHandler::goalCallback, this);
        sub_victims_ = nh.subscribe("/victims", 1, &EnvironmentHandler::victimsCallback, this);
        
        std::string odom_topic = "/" + robot_name + "/odom";
        sub_odom_ = nh.subscribe(odom_topic, 1, &EnvironmentHandler::odomCallback, this);
        
        ROS_INFO("EnvHandler initialized.");
    }

    // --- GETTERS ---
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    const Obstacle& getMapBorders() const { return border_obstacle_; } 
    const Eigen::Vector3d& getStartPose() const { return start_pose_; }
    const Eigen::Vector3d& getGoalPose() const { return goal_pose_; }
    const std::vector<VictimData>& getVictims() const { return victims_; }    
    
    // Ready if we have Borders, Start, Goal and victims 
    bool isReady() const {
        bool all_ready = map_borders_ready_ && goal_ready_ && start_ready_ && obstacles_ready_ && victims_ready_;
        if(all_ready){
            ROS_INFO_ONCE("Map borders, obstacles, victims, start and goal ready.");} 
        else{
            ROS_INFO("Map borders, obstacles, victims, start and goal not ready yet.");
        }
        return all_ready;    
    }

    // Helper function to check occupancy
    bool checkOccupancy(double x, double y) const {
        // 1. Check Map Borders
        // Returns TRUE if the point is OUTSIDE the map borders (a collision)
        if (border_obstacle_.checkCollision(x, y)) {
            return true; 
        }

        // 2. Check Obstacles
        // Returns TRUE if the point is INSIDE an inflated obstacle (a collision)
        for (const auto& obs : obstacles_) {
            if (obs.checkCollision(x, y)) {
                return true;
            }
        }

        return false; // Point is in free space
    }

    // Random sampler std::uniform_real_distribution generates points in a rectangular grid,
    // we must use a bounding box to "box in" the map hexagon,
    // and then use collision check to "trim" away the points that fall outside the hexagon
    double getMapWidth() const {
        // Distance between the furthest X points of the hexagon
        return border_obstacle_.getMaxX() - border_obstacle_.getMinX();
    }

    double getMapHeight() const {
        // Distance between the furthest Y points of the hexagon
        return border_obstacle_.getMaxY() - border_obstacle_.getMinY();
    }

    // Add these to EnvironmentHandler public section
    double getMinX() const { return border_obstacle_.getMinX(); }
    double getMaxX() const { return border_obstacle_.getMaxX(); }
    double getMinY() const { return border_obstacle_.getMinY(); }
    double getMaxY() const { return border_obstacle_.getMaxY(); }

private:
    // --- CALLBACKS ---
    
    // Note: Removed "PlannerBase::" prefix. We are in EnvironmentHandler now.
    void mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
        if (map_borders_ready_) return;

        border_obstacle_ = Obstacle(*msg);

        // Use the pre-calculated inflation!
        // Note the MINUS sign for borders (inflating inward)
        border_obstacle_.computeCSpace(total_inflation_, inflation_epsilon_); 

        visualizer_.publishBorders(border_obstacle_);
        
        map_borders_ready_ = true;
        ROS_INFO("Map borders processed.");
    }

    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        obstacles_.clear();

        for (const auto& obs_msg : msg->obstacles) {
            Obstacle obs(obs_msg);
            
            // Use pre-calculated inflation
            obs.computeCSpace(total_inflation_, inflation_epsilon_);

            obstacles_.push_back(obs);
        }

        obstacles_ready_ = true;
        visualizer_.publishObstacles(obstacles_);

        sub_obs_.shutdown(); // This kills the subscription, as obstacles are fixed so it's not needed anymore
        ROS_INFO("Obstacles received. Unsubscribing to save resources.");
    }

    void goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (goal_ready_ || msg->poses.empty()) return;

        const auto& goal = msg->poses[0];
        
        tf2::Quaternion q(
            goal.orientation.x,
            goal.orientation.y,
            goal.orientation.z,
            goal.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        goal_pose_ = Eigen::Vector3d(goal.position.x, goal.position.y, yaw);
        goal_ready_ = true;
        ROS_INFO("Goal set: [%f, %f, %f]", goal_pose_.x(), goal_pose_.y(), goal_pose_.z());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (start_ready_) return; 

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        start_pose_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        start_ready_ = true;
        ROS_INFO("Start pose set: [%f, %f, %f]", start_pose_.x(), start_pose_.y(), start_pose_.z());
    }

    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        victims_.clear();
        for (const auto& obs : msg->obstacles) {
            VictimData v;
            v.position = Eigen::Vector2d(obs.polygon.points[0].x, obs.polygon.points[0].y);
            v.reward = obs.radius; // This is the weight/score
            victims_.push_back(v);
        }
        victims_ready_ = true;
    }   

    // --- DATA ---
    double robot_radius_;
    double safety_margin_;
    double inflation_epsilon_;
    double total_inflation_; // Stored for efficiency

    std::vector<Obstacle> obstacles_;
    Obstacle border_obstacle_;
    Eigen::Vector3d start_pose_;
    Eigen::Vector3d goal_pose_;
    std::vector<VictimData> victims_;

    bool map_borders_ready_; // Renamed from map_ready_ to be specific
    bool obstacles_ready_;
    bool goal_ready_;
    bool start_ready_;
    bool victims_ready_;

    ros::Subscriber sub_borders_, sub_obs_, sub_goal_, sub_odom_, sub_victims_;
    
    // The visualizer now lives here, because this class owns the data!
    PlannerVisualizer visualizer_; 
};

#endif
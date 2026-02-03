#include "environment/EnvironmentHandler.hpp"

EnvironmentHandler::EnvironmentHandler() 
    : map_borders_ready_(false), 
      obstacles_ready_(false), 
      goal_ready_(false), 
      start_ready_(false),
      victims_ready_(false) {}

void EnvironmentHandler::init(ros::NodeHandle& nh, const std::string& robot_name) {
    ros::NodeHandle pnh("~"); 

    pnh.param<double>("robot_radius", robot_radius_, 0.2);
    pnh.param<double>("safety_margin", safety_margin_, 0.05);
    pnh.param<double>("inflation_epsilon", inflation_epsilon_, 0.01);

    total_inflation_ = robot_radius_ + safety_margin_;

    sub_borders_ = nh.subscribe("/map_borders", 1, &EnvironmentHandler::mapBordersCallback, this);
    sub_obs_ = nh.subscribe("/obstacles", 1, &EnvironmentHandler::obstaclesCallback, this);
    sub_goal_ = nh.subscribe("/gates", 1, &EnvironmentHandler::goalCallback, this);
    sub_victims_ = nh.subscribe("/victims", 1, &EnvironmentHandler::victimsCallback, this);
    
    std::string odom_topic = "/" + robot_name + "/odom";
    sub_odom_ = nh.subscribe(odom_topic, 1, &EnvironmentHandler::odomCallback, this);
    
    ROS_INFO("EnvHandler initialized.");
}

bool EnvironmentHandler::isReady() const {
    bool all_ready = map_borders_ready_ && goal_ready_ && start_ready_ && obstacles_ready_ && victims_ready_;
    if(all_ready){
        ROS_INFO_ONCE("Map borders, obstacles, victims, start and goal ready.");
    } else {
        ROS_INFO_THROTTLE(5, "Environment data not fully ready yet...");
    }
    return all_ready;    
}

bool EnvironmentHandler::checkOccupancy(double x, double y) const {
    if (border_obstacle_.checkCollision(x, y)) return true; 
    for (const auto& obs : obstacles_) {
        if (obs.checkCollision(x, y)) return true;
    }
    return false;
}

double EnvironmentHandler::getMapWidth() const {
    return border_obstacle_.getMaxX() - border_obstacle_.getMinX();
}

double EnvironmentHandler::getMapHeight() const {
    return border_obstacle_.getMaxY() - border_obstacle_.getMinY();
}

void EnvironmentHandler::mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
    if (map_borders_ready_) return;
    border_obstacle_ = Obstacle(*msg);
    border_obstacle_.computeCSpace(total_inflation_, inflation_epsilon_); 
    visualizer_.publishBorders(border_obstacle_);
    map_borders_ready_ = true;
    ROS_INFO("Map borders processed.");
}

void EnvironmentHandler::obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
    obstacles_.clear();
    for (const auto& obs_msg : msg->obstacles) {
        Obstacle obs(obs_msg);
        obs.computeCSpace(total_inflation_, inflation_epsilon_);
        obstacles_.push_back(obs);
    }
    obstacles_ready_ = true;
    visualizer_.publishObstacles(obstacles_);
    sub_obs_.shutdown();
    ROS_INFO("Obstacles received and processed.");
}

void EnvironmentHandler::goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (goal_ready_ || msg->poses.empty()) return;
    const auto& goal = msg->poses[0];
    tf2::Quaternion q(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    goal_pose_ = Eigen::Vector3d(goal.position.x, goal.position.y, yaw);
    goal_ready_ = true;
    ROS_INFO("Goal set: [%.2f, %.2f]", goal_pose_.x(), goal_pose_.y());
}

void EnvironmentHandler::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (start_ready_) return; 
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    start_pose_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    start_ready_ = true;
    ROS_INFO("Start pose set: [%.2f, %.2f]", start_pose_.x(), start_pose_.y());
}

void EnvironmentHandler::victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
    victims_.clear();
    for (const auto& obs : msg->obstacles) {
        VictimData v;
        v.position = Eigen::Vector2d(obs.polygon.points[0].x, obs.polygon.points[0].y);
        v.reward = obs.radius;
        victims_.push_back(v);
    }
    victims_ready_ = true;
}
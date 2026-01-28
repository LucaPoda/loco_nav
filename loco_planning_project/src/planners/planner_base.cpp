#include "planners/planner_base.hpp"
#include <tf2/utils.h> // For quaternion to yaw

#include "utils/obstacles.hpp"
#include "utils/planner_visualizer.hpp"

PlannerBase::PlannerBase(const std::string& robot_name) 
    : robot_name_(robot_name), 
      map_ready_(false), 
      obstacles_ready_(false), 
      goal_ready_(false), 
      start_ready_(false), 
      path_computed_(false),
      visualizer_(PlannerVisualizer()) 
{
    // 1. Handle Node Handles
    // 'nh_' is for global topics (e.g. /obstacles)
    // 'pnh' is for private parameters (e.g. ~v_max)
    ros::NodeHandle pnh("~");

    // 2. Allow overriding the robot_name via launch file if needed
    pnh.param<std::string>("robot_name", robot_name_, robot_name);

    ROS_INFO("Initializing Planner for Robot: %s", robot_name_.c_str());

    // ---------------------------------------------------------
    // STRATEGY: SYNC CONFIGURATION
    // ---------------------------------------------------------

    // 3. LOAD DT (CRITICAL)
    // In Python: dt = conf.robot_params[self.robot_name]['dt']
    // We try to find '/limo0/dt' on the server. 
    // If the Python controller loaded params.py to ROS, it will be here.
    std::string global_dt_param = "/" + robot_name_ + "/dt";
    
    if (nh_.hasParam(global_dt_param)) {
        nh_.getParam(global_dt_param, params_.dt);
        ROS_INFO("Synced 'dt' from global config: %.4f s", params_.dt);
    } else {
        // Fallback: Read from our own planning.yaml
        // YOU MUST ENSURE THIS MATCHES params.py MANUALLY
        pnh.param<double>("dt", params_.dt, 0.01); 
        ROS_WARN("Global '%s' not found. Using local 'dt': %.4f. ENSURE THIS MATCHES params.py!", 
                 global_dt_param.c_str(), params_.dt);
    }

    // 4. LOAD ROBOT PHYSICAL LIMITS
    // These depend on the specific robot (limo0, limo1...)
    // In your Python code, these were passed as arguments to __init__, 
    // so we load them from our private config (planning.yaml).

    // Robot Radius (Physical dimension)
    pnh.param<double>("robot_radius", params_.robot_radius, 0.2);

    // Max Velocity (Physical/Safety limit)
    pnh.param<double>("v_max", params_.v_max, 0.1);

    // Max Curvature (Steering limit)
    // Note: Python default was 3.0, but your params.py snippet doesn't show it explicitly.
    // If it's a physical property, it should be in the config.
    pnh.param<double>("curvature_max", params_.curvature_max, 3.0);
    
    // Safety Margin (Virtual buffer)
    pnh.param<double>("safety_margin", safety_margin_, 0.05);

    // ---------------------------------------------------------
    // LOGGING
    // ---------------------------------------------------------
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  CONFIGURATION SUMMARY");
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  DT (Time Step):    %.4f s", params_.dt);
    ROS_INFO("  Robot Radius:      %.3f m", params_.robot_radius);
    ROS_INFO("  Safety Margin:     %.3f m", safety_margin_);
    ROS_INFO("  Planning Radius:   %.3f m", params_.robot_radius + safety_margin_);
    ROS_INFO("  V Max:             %.2f m/s", params_.v_max);
    ROS_INFO("  Curvature Max:     %.2f (Rad: %.2fm)", params_.curvature_max, 1.0/params_.curvature_max);
    ROS_INFO("------------------------------------------------");
}

PlannerBase::~PlannerBase() {
    // Cleanup if necessary
}

void PlannerBase::init() {
    // Publishers
    std::string ref_topic = "/" + robot_name_ + "/ref";
    ref_pub_ = nh_.advertise<loco_planning::Reference>(ref_topic, 10);

    // Subscribers
    sub_borders_ = nh_.subscribe("/map_borders", 1, &PlannerBase::mapBordersCallback, this);
    sub_obs_ = nh_.subscribe("/obstacles", 1, &PlannerBase::obstaclesCallback, this);
    sub_goal_ = nh_.subscribe("/gates", 1, &PlannerBase::goalCallback, this);
    
    // In C++, getting initial state is robustly done via Odom subscriber rather than one-shot service
    std::string odom_topic = "/" + robot_name_ + "/odom";
    sub_odom_ = nh_.subscribe(odom_topic, 1, &PlannerBase::odomCallback, this);

    ROS_INFO("PlannerBase initialized for robot: %s", robot_name_.c_str());
}

void PlannerBase::run() {
    ros::Rate rate(1.0 / params_.dt);

    while (ros::ok()) {
        ros::spinOnce();

        // Check prerequisites
        if (!path_computed_ && map_ready_ && obstacles_ready_ && goal_ready_ && start_ready_) {
            ROS_INFO("All data received. Planning path...");
            
            // 1. CALL THE VIRTUAL PLANNER (Implemented by subclass)
            std::vector<Eigen::Vector3d> geometric_path = planPath();

            if (!geometric_path.empty()) {
                // 2. Compute Trajectory (Smoothing/Dubins)
                auto reference_traj = computeReferenceFromPath(geometric_path);

                // 3. Publish
                publishReference(reference_traj);
                
                path_computed_ = true; // Stop planning once done
            } else {
                ROS_WARN_THROTTLE(5, "Planner failed to find a path.");
            }
        }
        
        rate.sleep();
    }
}

// ---------------------------------------------------------
// COMMON TRAJECTORY LOGIC
// ---------------------------------------------------------

std::vector<loco_planning::Reference> PlannerBase::computeReferenceFromPath(const std::vector<Eigen::Vector3d>& path) {
    std::vector<loco_planning::Reference> full_reference;
    
    if (path.size() < 2) return full_reference;

    // Calculate step size in meters based on max velocity and time step
    // This ensures the trajectory points are spaced correctly in time
    double step_size_meters = params_.v_max * params_.dt;

    // Iterate through every pair of waypoints (Start -> Victim 1 -> Victim 2...)
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Eigen::Vector3d start = path[i];
        Eigen::Vector3d goal = path[i+1];

        // 1. Generate the discrete Dubins curve points using the OMPL wrapper
        // Note: start.z() and goal.z() must hold the orientation (theta)
        std::vector<Eigen::Vector3d> dubins_points = DubinsGenerator::getPath(
            start, 
            goal, 
            params_.curvature_max, 
            step_size_meters
        );

        // 2. Convert geometric points to Trajectory Reference messages
        for (size_t j = 0; j < dubins_points.size(); ++j) {
            
            // Optimization: Avoid duplicating points at the joints.
            // The last point of segment 'i' is the first point of segment 'i+1'.
            // We skip the first point of the new segment unless it's the very first segment.
            if (i > 0 && j == 0) continue;

            loco_planning::Reference ref;
            ref.x_d = dubins_points[j].x();
            ref.y_d = dubins_points[j].y();
            ref.theta_d = dubins_points[j].z(); // Z component holds Theta
            
            ref.v_d = params_.v_max;

            // 3. Calculate Feedforward Angular Velocity (Omega)
            // Since OMPL returns positions, we approximate omega using finite difference
            // Omega = d_theta / dt
            if (j < dubins_points.size() - 1) {
                double next_theta = dubins_points[j+1].z();
                double curr_theta = dubins_points[j].z();
                double d_theta = next_theta - curr_theta;

                // Normalize angle difference to [-pi, pi] to handle wrapping
                while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
                while (d_theta < -M_PI) d_theta += 2.0 * M_PI;

                ref.omega_d = d_theta / params_.dt;
            } else {
                // For the very last point of a segment, assume constant velocity from previous
                if (!full_reference.empty()) {
                    ref.omega_d = full_reference.back().omega_d;
                } else {
                    ref.omega_d = 0.0;
                }
            }

            full_reference.push_back(ref);
        }
    }
    
    return full_reference;
}

void PlannerBase::publishReference(const std::vector<loco_planning::Reference>& reference) {
    ros::Rate pub_rate(1.0 / params_.dt);
    
    ROS_INFO("Publishing reference trajectory with %lu points...", reference.size());

    for (const auto& ref_pt : reference) {
        if (!ros::ok()) break;
        ref_pub_.publish(ref_pt);
        pub_rate.sleep();
    }

    // Send completion signal (optional, based on Python logic)
    loco_planning::Reference end_ref;
    end_ref.plan_finished = true;
    ref_pub_.publish(end_ref);
    
    ROS_INFO("Trajectory publication finished.");
}

// ---------------------------------------------------------
// CALLBACKS
// ---------------------------------------------------------

void PlannerBase::mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
    if (map_ready_) return;

    // 1. Convert msg to our Obstacle class
    // (Using the new constructor suggested in Part 1)
    border_obstacle_ = Obstacle(*msg);

    // 2. Inflate INWARD (Negative offset)
    double total_offset = params_.robot_radius + safety_margin_;
    
    // Note the MINUS sign here!
    border_obstacle_.computeCSpace(total_offset, 0.01); 

    // 3. Debug Visualization
    visualizer_.publishBorders(border_obstacle_);
    
    map_ready_ = true;
    ROS_INFO("Map borders processed. Total obstacles: %lu", obstacles_.size());
}

void PlannerBase::obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
    if (obstacles_ready_) return;

    // 1. Clear any existing obstacles (safety check)
    obstacles_.clear();

    // 2. Calculate the total inflation offset
    // Offset = Robot Radius (physical) + Safety Margin (buffer for Dubins tracking error)
    // We assume 'safety_margin_' was loaded in init() from the yaml file
    double total_offset = params_.robot_radius + safety_margin_;

    // 3. Process each obstacle
    for (const auto& obs_msg : msg->obstacles) {
        // A. Instantiate the Obstacle
        // The constructor automatically parses whether it's a Circle or Polygon
        Obstacle obs(obs_msg);

        // B. Compute Configuration Space immediately
        // This runs Clipper2 to generate the "Rounded Polygon" with arcs
        // We use a default arc tolerance of 0.01m (1cm), or you can make it a parameter
        obs.computeCSpace(total_offset, 0.01);

        // C. Store in our vector
        obstacles_.push_back(obs);
    }

    obstacles_ready_ = true;

    visualizer_.publishObstacles(obstacles_);

    ROS_INFO("Obstacles received: %lu obstacles processed and inflated with offset %.3f m.", obstacles_.size(), total_offset);
}

void PlannerBase::goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (goal_ready_ || msg->poses.empty()) return;

    const auto& goal = msg->poses[0];
    
    // Orientation from Quaternion to Yaw
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

void PlannerBase::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (start_ready_) return; // Only grab start once

    // Orientation from Quaternion to Yaw
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

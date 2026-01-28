#include "planners/planner_base.hpp"
#include <tf2/utils.h> 

#include "utils/dubins_ompl.hpp" 

// INITIALIZATION
void PlannerBase::initialize(const std::string& robot_name) {
    robot_name_ = robot_name;
    
    // Create NodeHandles
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~");

    ROS_INFO("Initializing PlannerBase for Robot: %s", robot_name_.c_str());

    // Environment init:  sets up subscribers for Obstacles, Start, and Goal; loads robot_radius and safety_margin.
    env_.init(nh, robot_name_);

    // Loads DT from global params first and from private params if not found
    std::string global_dt_param = "/" + robot_name_ + "/dt";
    
    if (nh.hasParam(global_dt_param)) {
        nh.getParam(global_dt_param, params_.dt);
        ROS_INFO("Synced 'dt' from global config: %.4f s", params_.dt);
    } else {
        // Fallback: Read from our local planning.yaml
        pnh.param<double>("dt", params_.dt, 0.01); 
        ROS_WARN("Global '%s' not found. Using local 'dt': %.4f. ENSURE THIS MATCHES params.py!", 
                 global_dt_param.c_str(), params_.dt);
    }

    // 2. Load Trajectory Limits
    pnh.param<double>("v_max", params_.v_max, 0.5);
    pnh.param<double>("curvature_max", params_.curvature_max, 1.0);

    // Setup publishers
    std::string ref_topic = "/" + robot_name_ + "/ref";
    ref_pub_ = nh.advertise<loco_planning::Reference>(ref_topic, 10);

    // Log Configuration
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  PLANNER CONFIGURATION");
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  DT:             %.4f s", params_.dt);
    ROS_INFO("  V Max:          %.2f m/s", params_.v_max);
    ROS_INFO("  Curvature Max:  %.2f", params_.curvature_max);
    ROS_INFO("------------------------------------------------");
}

// Main Loop
void PlannerBase::run() {
    ros::Rate rate(1.0 / params_.dt);

    while (ros::ok()) {
        ros::spinOnce(); // Updates EnvironmentHandler callbacks

        // Check if we are ready to plan and haven't done so yet
        if (!path_computed_ && env_.isReady()) {
            
            ROS_INFO("Environment Ready. Calling Plugin Logic...");


            // Plan Geometric Path using a specific planner from plugin
            std::vector<Eigen::Vector3d> geometric_path = planPath();

            if (!geometric_path.empty()) {
                ROS_INFO("Path found with %lu waypoints. Generating trajectory...", geometric_path.size());


                // Compute reference trajectory from the geometric path
                auto reference_traj = computeReferenceFromPath(geometric_path);

                // Publish the trajectory to the topic
                publishReference(reference_traj);
                
                path_computed_ = true; // Stop planning (One-shot mission)
            } else {
                ROS_WARN_THROTTLE(5, "Planner Plugin returned an empty path.");
            }
        } else if (!env_.isReady()) {
            ROS_INFO_THROTTLE(5, "Waiting for data...");
        }
        
        rate.sleep();
    }
}

// converts the geometric trajectory to the reference trajectory
std::vector<loco_planning::Reference> PlannerBase::computeReferenceFromPath(const std::vector<Eigen::Vector3d>& path) {
    std::vector<loco_planning::Reference> full_reference;
    
    if (path.size() < 2) return full_reference;

    double step_size_meters = params_.v_max * params_.dt;

    // Iterate through waypoints
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Eigen::Vector3d start = path[i];
        Eigen::Vector3d goal = path[i+1];

        // Generate Dubins curve points
        std::vector<Eigen::Vector3d> dubins_points = DubinsGenerator::getPath(
            start, 
            goal, 
            params_.curvature_max, 
            step_size_meters
        );

        for (size_t j = 0; j < dubins_points.size(); ++j) {
            // Avoid duplicate points at joints
            if (i > 0 && j == 0) continue;

            loco_planning::Reference ref;
            ref.x_d = dubins_points[j].x();
            ref.y_d = dubins_points[j].y();
            ref.theta_d = dubins_points[j].z(); 
            ref.v_d = params_.v_max;

            // Calculate Omega (Feedforward)
            if (j < dubins_points.size() - 1) {
                double next_theta = dubins_points[j+1].z();
                double curr_theta = dubins_points[j].z();
                double d_theta = next_theta - curr_theta;

                // Normalize angle
                while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
                while (d_theta < -M_PI) d_theta += 2.0 * M_PI;

                ref.omega_d = d_theta / params_.dt;
            } else {
                // Last point of segment
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

    // Send completion signal
    loco_planning::Reference end_ref;
    end_ref.plan_finished = true;
    ref_pub_.publish(end_ref);
    
    ROS_INFO("Trajectory publication finished.");
}
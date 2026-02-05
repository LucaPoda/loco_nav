#include "planners/planner_base.hpp"
#include <tf2/utils.h> 

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

    pnh.param<double>("t_max", params_.t_max, 400.0);
    pnh.param<double>("max_shortcut_distance", params_.max_shortcut_distance, 1.0);
    pnh.param<double>("min_radius", params_.min_radius, 0.4);
    pnh.param<double>("step_size", params_.step_size, 0.05);

    // Setup publishers
    std::string ref_topic = "/" + robot_name_ + "/ref";
    ref_pub_ = nh.advertise<loco_planning::Reference>(ref_topic, 10, true);

    // Log Configuration
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  PLANNER CONFIGURATION");
    ROS_INFO("------------------------------------------------");
    ROS_INFO("  DT:                    %.4f s", params_.dt);
    ROS_INFO("  V Max:                 %.2f m/s", params_.v_max);
    ROS_INFO("  Curvature Max:         %.2f", params_.curvature_max);
    ROS_INFO("  t_max:                 %.2f", params_.t_max);
    ROS_INFO("  max_shortcut_distance: %.2f", params_.max_shortcut_distance);
    ROS_INFO("  min_radius:            %.2f", params_.min_radius);
    ROS_INFO("  step_size:             %.2f", params_.step_size);
    ROS_INFO("------------------------------------------------");
}

// Main Loop
void PlannerBase::run() {
    ros::Rate rate(1.0 / params_.dt);
    bool roadmap_built = false; // Global or class member
    Roadmap roadmap;
    std::map<int, std::map<int, double>> cost_matrix;

    while (ros::ok()) {
        ros::spinOnce(); // Updates EnvironmentHandler callbacks

        // Check if we are ready to plan and haven't done so yet
        if (!path_computed_ && env_.isReady()) {
            
            ROS_INFO_ONCE("Environment Ready. Calling Planner Logic...");

            // Plan Geometric Path using a specific planner from plugin
            
            if (!roadmap_built) {
                ROS_INFO("Building Roadmap...");

                // 1. Generate the dense roadmap
                roadmap = buildRoadmap();
                roadmap_built = true; // <--- This stops the infinite loop!
                
                // 2. Compute the Cost Matrix (All-pairs Dijkstra for special nodes)
                ROS_INFO("Computing Special Nodes Matrix...");
                
                // Initialize special_ids
                for (size_t i=0; i<env_.getVictims().size()+2; i++) {
                    special_ids.push_back(i);
                }

                // 3. Build Distance Matrix from roadmap
                DistanceMatrix distance_matrix(roadmap, special_ids);

                ROS_INFO_STREAM(distance_matrix.display().str() << "\n-----------------------");

                // 4. Build a simplified roadmap with only the shortest paths
                Roadmap simplified_roadmap = distance_matrix.buildShortestPathsRoadmap();

                // 5: Plan the best sequence of victims to visit on the simplified roadmap
                std::vector<int> node_sequence = OrienteeringPlanner::plan(simplified_roadmap, special_ids[0], special_ids[1], params_.t_max);

                // 6: Reconstruct the full path from the victims sequence
                std::vector<int> full_path = distance_matrix.getFullPath(node_sequence);

                // 7: Compute a smoothed trajectory using short cutting
                std::vector<int> smoothed_path = smoothPathVictimAware(roadmap, full_path, params_.max_shortcut_distance);

                // 8: Compute the Dubins trajectory from the smoothed path
                std::vector<TrajectoryPoint> dubins_trajectory = computeOMPLDubinsTrajectory(roadmap, smoothed_path, params_.min_radius, params_.step_size);

                // 9: Compute the final reference trajectory from the Dubins path
                auto reference_traj = computeReferenceFromPath(dubins_trajectory);

                // 10. Publish the reference trajectory
                publishReference(reference_traj);

                visualizer_.publishRoadmap(roadmap);
                visualizer_.publishDistanceMatrix(simplified_roadmap, distance_matrix);
                visualizer_.publishOrienteeringPath(roadmap, full_path);
                visualizer_.publishSmoothedPath(roadmap, smoothed_path);
                visualizer_.publishDubinsTrajectory(dubins_trajectory);

                ROS_INFO("Roadmap and High-Level Matrix ready.");
            }
        } else if (!env_.isReady()) {
            ROS_INFO_THROTTLE(5, "Waiting for data (Start, Goal, Borders, Obstacles)...");
        }
        
        rate.sleep();
    }
}

std::vector<loco_planning::Reference> PlannerBase::computeReferenceFromPath(const std::vector<TrajectoryPoint>& dubins_trajectory) {
    std::vector<loco_planning::Reference> full_reference;
    
    // Need at least two points to calculate angular velocity
    if (dubins_trajectory.size() < 2) return full_reference;

    for (size_t i = 0; i < dubins_trajectory.size(); ++i) {
        loco_planning::Reference ref;
        
        // 1. Assign Position and Heading
        ref.x_d = dubins_trajectory[i].x;
        ref.y_d = dubins_trajectory[i].y;
        ref.theta_d = dubins_trajectory[i].theta; 
        
        // 2. Assign Constant Linear Velocity
        ref.v_d = params_.v_max;

        // 3. Calculate Angular Velocity (Omega) via Finite Difference
        if (i < dubins_trajectory.size() - 1) {
            double next_theta = dubins_trajectory[i+1].theta;
            double curr_theta = dubins_trajectory[i].theta;
            double d_theta = next_theta - curr_theta;

            // Normalize angle difference to [-PI, PI] to handle wrap-around
            while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
            while (d_theta < -M_PI) d_theta += 2.0 * M_PI;

            // Omega = delta_theta / delta_t
            ref.omega_d = d_theta / params_.dt;
        } else {
            // Last point: Maintain the previous omega or set to zero
            if (!full_reference.empty()) {
                ref.omega_d = full_reference.back().omega_d;
            } else {
                ref.omega_d = 0.0;
            }
        }

        full_reference.push_back(ref);
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


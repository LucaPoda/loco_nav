#include "planners/planner_base.hpp"
#include <tf2/utils.h> 

#include <iomanip> // For std::setw, std::setprecision
#include <sstream> // For std::stringstream

// INITIALIZATION
void PlannerBase::initialize(const std::string& robot_name) {
    robot_name_ = robot_name;
    // Create NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("Initializing PlannerBase for Robot: %s", robot_name_.c_str());

    // Environment init
    env_.init(nh, robot_name_);

    // 1. Loads DT
    std::string global_dt_param = "/" + robot_name_ + "/dt";
    if (nh.hasParam(global_dt_param)) {
        nh.getParam(global_dt_param, params_.dt);
        ROS_INFO("Synced 'dt' from global config: %.4f s", params_.dt);
    } else {
        pnh.param<double>("dt", params_.dt, 0.001);
        ROS_WARN("Global '%s' not found. Using local 'dt': %.4f", global_dt_param.c_str(), params_.dt);
    }

    // 2. Load Trajectory Limits
    pnh.param<double>("v_max", params_.v_max, 0.5);
    pnh.param<double>("curvature_max", params_.curvature_max, 1.0);
    
    pnh.param<double>("min_radius", params_.min_radius, 0.4);
    pnh.param<double>("step_size", params_.step_size, 0.05);


    // --- NEW: READ TIMEOUT FROM GLOBAL PARAM ---
    std::string timeout_param = "/_/send_timeout/ros__parameters/victims_timeout";
    if (nh.hasParam(timeout_param)) {
        int timeout_int;
        // Often these specific params are integers (seconds), so we read as int and cast
        // If it's a float/double in the param server, use nh.getParam(timeout_param, params_.t_max) directly.
        nh.getParam(timeout_param, timeout_int); 
        params_.t_max = static_cast<double>(timeout_int);
        ROS_INFO("Synced 't_max' from global timeout param: %.2f s", params_.t_max);
    } else {
        // Fallback to local param if the topic/param doesn't exist
        pnh.param<double>("t_max", params_.t_max, 400.0);
        ROS_WARN("Global timeout param '%s' not found. Using local default: %.2f s", timeout_param.c_str(), params_.t_max);
    }
    // -------------------------------------------

    max_path_length_ = params_.t_max * params_.v_max;

    // Setup publishers
    std::string ref_topic = "/" + robot_name_ + "/ref";
    ref_pub_ = nh.advertise<loco_planning::Reference>(ref_topic, 10);

    // Log Configuration
    ROS_INFO("------------------------------------------------");
    ROS_INFO(" PLANNER CONFIGURATION");
    ROS_INFO("------------------------------------------------");
    ROS_INFO(" DT: %.4f s", params_.dt);
    ROS_INFO(" V Max: %.2f m/s", params_.v_max);
    ROS_INFO(" Curvature Max: %.2f", params_.curvature_max);
    ROS_INFO(" t_max (Timeout): %.2f s", params_.t_max); // Updated label
    ROS_INFO(" s_max (Path Length): %.2f m", max_path_length_); // Updated label
    ROS_INFO(" min_radius: %.2f", params_.min_radius);
    ROS_INFO(" step_size: %.2f", params_.step_size);
    ROS_INFO("------------------------------------------------");
}

void PlannerBase::run() {
    ros::Rate rate(1.0 / params_.dt);
    bool roadmap_built = false; 
    Roadmap roadmap;
    std::vector<int> special_ids; // Needs to be persistent for the loop

    std::vector<int> full_path;
    std::vector<int> smoothed_path;

    while (ros::ok()) {
        ros::spinOnce(); 
        
        if (!path_computed_ && env_.isReady()) {
            
            ROS_INFO_ONCE("Environment Ready. Calling Planner Logic...");
            
            if (!roadmap_built) {
                // --- START TIMER ---
                ros::WallTime start_time = ros::WallTime::now();

                ROS_INFO("Building Roadmap...");

                // 1. Generate the dense roadmap
                roadmap = buildRoadmap();
                roadmap_built = true;
                visualizer_.publishRoadmap(roadmap);
                
                // 2. Compute Special Nodes
                ROS_INFO("Computing Special Nodes Matrix...");
                special_ids.clear();
                for (size_t i=0; i<env_.getVictims().size()+2; i++) {
                    special_ids.push_back(i);
                }

                // 3. Build Distance Matrix (Computed ONCE)
                DistanceMatrix distance_matrix(roadmap, special_ids);

                // 4. Build simplified roadmap (Computed ONCE)
                Roadmap simplified_roadmap = distance_matrix.buildShortestPathsRoadmap();
                // visualizer_.publishDistanceMatrix(simplified_roadmap, distance_matrix);
                
                // ---------------------------------------------------------
                // 5. ITERATIVE PLANNING LOOP (The "Repair" Logic)
                // ---------------------------------------------------------
                std::set<int> blacklisted_victims;
                
                // Backup original scores so we can restore them if needed
                std::map<int, double> original_scores;
                for(const auto& node_pair : simplified_roadmap.getNodes()) {
                    original_scores[node_pair.first] = node_pair.second.score;
                }

                bool plan_found = false;
                std::vector<TrajectoryPoint> final_dubins_trajectory;
                int max_retries = 10;

                for (int attempt = 0; attempt < max_retries; ++attempt) {
                    
                    // A. Apply Blacklist: Set scores of bad nodes to 0 so Orienteering ignores them
                    for (auto& node_pair : simplified_roadmap.getNodes()) {
                        if (blacklisted_victims.count(node_pair.first)) {
                            node_pair.second.score = 0.0; 
                        } else {
                            node_pair.second.score = original_scores[node_pair.first]; 
                        }
                    }

                    ROS_INFO("Update nodes blacklist:");
                    for (auto& node_pair : simplified_roadmap.getNodes()) {
                        ROS_INFO("Special-Node: %d => %f", node_pair.first, node_pair.second.score);
                    }

                    // B. Plan Sequence (Start Fresh)
                    // Calculate effective distance budget (Time * Speed * SafetyFactor) 
                    std::vector<int> node_sequence = OrienteeringPlanner::plan(simplified_roadmap, special_ids[0], special_ids[1], max_path_length_);

                    // C. Reconstruct & Smooth Path
                    full_path = distance_matrix.getFullPath(node_sequence);
                    
                    smoothed_path = smoothPathVictimAware(roadmap, full_path, env_, params_.dt);

                    // D. Compute Dubins (Using new Forward-Fail logic)
                    // Note: Ensure params_.min_radius matches your config (e.g., 0.4)
                    auto result = computeOMPLDubinsTrajectory(roadmap, smoothed_path, 
                                                            params_.curvature_max, params_.dt,
                                                            env_.getStartPose().z(), env_.getGoalPose().z(), env_);

                    if (result.success) {
                        // --- SUCCESS! ---
                        final_dubins_trajectory = result.trajectory;
                        
                        
                        visualizer_.publishDubinsTrajectory(final_dubins_trajectory);
                        
                        ROS_INFO("Valid Dubins plan found on attempt %d with %lu victims.", attempt + 1, node_sequence.size() - 2);
                        plan_found = true;
                        break; 
                    } 
                    else {
                        // --- FAILURE: Identify and Blacklist ---
                        // The robot is valid up to: smoothed_path[result.last_valid_idx]
                        // We need to find the first VICTIM that appears AFTER this point.
                        
                        int unreachable_victim_id = -1;

                        // 1. Scan forward from the failure point to find the next Victim
                        for (size_t k = result.last_valid_idx + 1; k < smoothed_path.size(); ++k) {
                            int next_node = smoothed_path[k];
                            
                            // Check if 'next_node' is a target victim 
                            // (We iterate s from 1 to size-2 to exclude Start and Goal)
                            bool is_target_victim = false;
                            for (size_t s = 1; s < node_sequence.size() - 1; ++s) {
                                if (node_sequence[s] == next_node) {
                                    is_target_victim = true; 
                                    break;
                                }
                            }

                            if (is_target_victim) {
                                unreachable_victim_id = next_node;
                                break; // Found the specific victim we couldn't reach
                            }
                        }

                        // 2. Fallback: If no victim was found, the failure occurred on the final leg to the Goal.
                        // We cannot blacklist the Goal, so we blacklist the LAST victim in the sequence
                        // to force the planner to try a different approach (or skip that victim).
                        if (unreachable_victim_id == -1 && node_sequence.size() > 2) {
                            unreachable_victim_id = node_sequence[node_sequence.size() - 2];
                            ROS_WARN("Dubins failure on leg to Goal. Blaming last victim %d.", unreachable_victim_id);
                        }

                        // 3. Apply Blacklist
                        if (unreachable_victim_id != -1) {
                            ROS_WARN("Dubins failed to reach victim %d. Blacklisting and Retrying...", unreachable_victim_id);
                            blacklisted_victims.insert(unreachable_victim_id);
                        } else {
                            ROS_ERROR("Dubins failed but could not identify a victim to prune. Aborting.");
                            break;
                        }
                    }
                }
                
                // Visualize one time after the loop
                visualizer_.publishOrienteeringPath(roadmap, full_path);
                visualizer_.publishSmoothedPath(roadmap, smoothed_path);

                // ---------------------------------------------------------

                // 6. Compute Reference (only if plan found)
                std::vector<loco_planning::Reference> reference_traj;
                if (plan_found) {
                    reference_traj = computeReferenceFromPath(final_dubins_trajectory);
                }

                // --- STOP TIMER & LOGGING ---
                double computation_time = (ros::WallTime::now() - start_time).toSec();
                double length_m = 0.0;
                if (!final_dubins_trajectory.empty()) {
                    for (size_t i = 0; i < final_dubins_trajectory.size() - 1; ++i) {
                        length_m += std::hypot(final_dubins_trajectory[i+1].x - final_dubins_trajectory[i].x,
                                               final_dubins_trajectory[i+1].y - final_dubins_trajectory[i].y); 
                    }
                }

                std::stringstream ss;
                ss << "\n" << std::string(50, '-') << "\n";
                ss << " PLANNING METRICS \n";
                ss << std::string(50, '-') << "\n";
                ss << std::left << std::setw(30) << "Computation Time (s)" << std::right << std::setw(15) << std::fixed << std::setprecision(4) << computation_time << "\n";
                ss << std::left << std::setw(30) << "Trajectory Length (m)" << std::right << std::setw(15) << std::fixed << std::setprecision(2) << length_m << "\n";
                ss << std::left << std::setw(30) << "Victims Visited" << std::right << std::setw(15) << (plan_found ? std::to_string(original_scores.size() - 2 - blacklisted_victims.size()) : "FAILED") << "\n";
                ss << std::string(50, '-') << "\n";
                ROS_INFO_STREAM(ss.str());

                // 7. Publish
                if (plan_found) {
                    publishReference(reference_traj);
                } else {
                    ROS_ERROR("Failed to find any valid plan after retries.");
                }
                
                ROS_INFO("Roadmap and High-Level Matrix ready.");
            }
        } else if (!env_.isReady()) {
            ROS_INFO_THROTTLE(5, "Waiting for data...");
        }
        
        rate.sleep();
    }
}
std::vector<loco_planning::Reference> PlannerBase::computeReferenceFromPath(const std::vector<TrajectoryPoint>& dubins_trajectory) {
    std::vector<loco_planning::Reference> full_reference;
    if (dubins_trajectory.size() < 2) return full_reference;

    // FIX 1: Declare and initialize last_unwrapped_theta
    double last_unwrapped_theta = dubins_trajectory[0].theta; 

    for (size_t i = 0; i < dubins_trajectory.size(); ++i) {
        loco_planning::Reference ref;
        
        // 1. Maintain the Unwrapped Theta
        double current_raw_theta = dubins_trajectory[i].theta;
        double d_theta_raw = std::atan2(std::sin(current_raw_theta - last_unwrapped_theta), 
                                        std::cos(current_raw_theta - last_unwrapped_theta));
        double unwrapped_theta = last_unwrapped_theta + d_theta_raw;
        
        // Wrap for the published message to stay consistent with Python's wrap
        ref.theta_d = unwrapped_theta;
        last_unwrapped_theta = unwrapped_theta;

        ref.x_d = dubins_trajectory[i].x;
        ref.y_d = dubins_trajectory[i].y;
        ref.v_d = params_.v_max;

        // 2. Kinematic Consistency
        if (i < dubins_trajectory.size() - 1) {
            double next_theta = dubins_trajectory[i+1].theta;
            double current_theta = dubins_trajectory[i].theta;
            
            double delta_theta = std::atan2(std::sin(next_theta - current_theta), 
                                            std::cos(next_theta - current_theta));

            ref.omega_d = delta_theta / params_.dt; 
        } else {
            ref.omega_d = 0.0;
        }

        full_reference.push_back(ref);
    }

    // FIX 2: Return the vector!
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


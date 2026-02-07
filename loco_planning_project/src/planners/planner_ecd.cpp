// planner_ecd.cpp
#include "planners/planner_ecd.hpp"
#include <pluginlib/class_list_macros.h>
#include <limits>
#include <algorithm>

PlannerECD::PlannerECD() {}
PlannerECD::~PlannerECD() {}

void PlannerECD::initialize(const std::string& robot_name) {
    PlannerBase::initialize(robot_name);

    ros::NodeHandle nh;
    slab_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/ecd/slabs", 1, true
    );

    ROS_INFO("Initializing ECD Planner plugin...");
}

bool collisionFreeSegment(const EnvironmentHandler& env, const Eigen::Vector2d& a, const Eigen::Vector2d& b, double step = 0.02);

Roadmap PlannerECD::buildRoadmap() {
    Roadmap roadmap;
    const auto& env = getEnvironment();
    const auto& params = getParams();

    ROS_INFO("PlannerECD: building deterministic decomposition roadmap");

    std::vector<Eigen::Vector2d> node_positions;

    // ---- Special node IDs (must match PlannerBase expectations)
    const int START_ID = 0;
    const int GOAL_ID  = 1;
    int next_id = 2;

    // ---- Add start & goal
    Eigen::Vector2d start = env.getStartPose().head<2>();
    Eigen::Vector2d goal  = env.getGoalPose().head<2>();

    roadmap.addNode(START_ID, start, 0.0);
    roadmap.addNode(GOAL_ID,  goal,  0.0);

    // keep node_positions aligned with roadmap insertion order:
    node_positions.push_back(start); // index 0
    node_positions.push_back(goal);  // index 1

    const auto& victims = env.getVictims();

    // Victims: IDs 2 .. 2+V-1
    for (size_t i = 0; i < victims.size(); ++i) {
        int v_id = next_id++;
        roadmap.addNode(v_id, victims[i].position, victims[i].reward);
        node_positions.push_back(victims[i].position);
    }

    // ---- Map bounds
    double min_x = env.getMinX();
    double max_x = env.getMaxX();
    double min_y = env.getMinY();
    double max_y = env.getMaxY();

    // ---- Deterministic vertical slicing
    const int N_SLICES = 40;
    const int N_Y_SAMPLES = 80;

    struct Cell {
        int center_id;
        double xl, xr;
        double ylo, yhi;
        Eigen::Vector2d center;
    };

    std::vector<std::vector<Cell>> slabs; // slabs[i] -> vector of cells in slab i
    slabs.resize(std::max(0, N_SLICES));

    visualization_msgs::MarkerArray slab_markers;
    int marker_id = 0;

    // Create center nodes (one per free interval per slab)
    for (int i = 0; i < N_SLICES; ++i) {
        double xl = min_x + ( (double)i      ) * (max_x - min_x) / N_SLICES;
        double xr = min_x + ( (double)(i+1) ) * (max_x - min_x) / N_SLICES;
        double xm = 0.5 * (xl + xr);

        bool in_free = false;
        double y_start = 0.0;

        for (int j = 0; j <= N_Y_SAMPLES; ++j) {
            double y = min_y + j * (max_y - min_y) / N_Y_SAMPLES;
            bool occ = env.checkOccupancy(xm, y);

            if (!occ && !in_free) {
                in_free = true;
                y_start = y;
            }
            if ((occ || j == N_Y_SAMPLES) && in_free) {
                double y_end = y;
                double height = y_end - y_start;
                double yc = 0.5 * (y_start + y_end);
                Eigen::Vector2d center_pos(xm, yc);

                // Store cell
                Cell c;
                c.xl = xl;
                c.xr = xr;
                c.ylo = y_start;
                c.yhi = y_end;
                c.center = center_pos;
                c.center_id = next_id++;
                slabs[i].push_back(c);

                // Add center node to roadmap
                roadmap.addNode(c.center_id, c.center, 0.0);
                node_positions.push_back(c.center);

                // ---- RViz slab marker (rectangle covering slab horizontal extent and this y-interval)
                visualization_msgs::Marker slab;
                slab.header.frame_id = "map";
                slab.header.stamp = ros::Time::now();
                slab.ns = "ecd_slabs";
                slab.id = marker_id++;
                slab.type = visualization_msgs::Marker::CUBE;
                slab.action = visualization_msgs::Marker::ADD;

                // Center the cube at the slab midpoint horizontally and interval midpoint vertically
                slab.pose.position.x = 0.5 * (xl + xr);
                slab.pose.position.y = yc;
                slab.pose.position.z = 0.02;
                slab.pose.orientation.w = 1.0;

                // Size: slightly narrower than slab width, and equal to interval height
                slab.scale.x = (xr - xl) * 0.95;
                slab.scale.y = height;
                slab.scale.z = 0.04;

                slab.color.r = 0.2;
                slab.color.g = 0.4;
                slab.color.b = 1.0;
                slab.color.a = 0.35;
                slab.lifetime = ros::Duration(0);
                slab_markers.markers.push_back(slab);

                // ---- RViz center node marker
                visualization_msgs::Marker node;
                node.header = slab.header;
                node.ns = "ecd_centers";
                node.id = marker_id++;
                node.type = visualization_msgs::Marker::SPHERE;

                node.pose.position.x = c.center.x();
                node.pose.position.y = c.center.y();
                node.pose.position.z = 0.06;

                node.scale.x = 0.08;
                node.scale.y = 0.08;
                node.scale.z = 0.08;

                node.color.r = 1.0;
                node.color.g = 0.2;
                node.color.b = 0.2;
                node.color.a = 1.0;

                slab_markers.markers.push_back(node);

                in_free = false;
            }
        }
    }

    // ---- Create edge nodes at overlap midpoints between slab i and i+1, and connect centers->edge nodes
    std::vector<int> edge_node_ids; // for possible future use/visualization
    for (int i = 0; i + 1 < N_SLICES; ++i) {
        const auto& left_cells = slabs[i];
        const auto& right_cells = slabs[i+1];

        for (size_t li = 0; li < left_cells.size(); ++li) {
            for (size_t ri = 0; ri < right_cells.size(); ++ri) {
                double a_lo = left_cells[li].ylo;
                double a_hi = left_cells[li].yhi;
                double b_lo = right_cells[ri].ylo;
                double b_hi = right_cells[ri].yhi;

                double overlap_lo = std::max(a_lo, b_lo);
                double overlap_hi = std::min(a_hi, b_hi);

                if (overlap_hi > overlap_lo + 1e-9) {
                    // Shared vertical edge x coordinate is at right boundary of left slab (or left boundary of right slab)
                    double x_shared = 0.5 * (left_cells[li].xr + right_cells[ri].xl); // should be same
                    double y_shared_mid = 0.5 * (overlap_lo + overlap_hi);
                    Eigen::Vector2d edge_pos(x_shared, y_shared_mid);

                    int edge_id = next_id++;
                    roadmap.addNode(edge_id, edge_pos, 0.0);
                    node_positions.push_back(edge_pos);
                    edge_node_ids.push_back(edge_id);

                    // Connect left center -> edge, edge -> left center
                    double wL = (left_cells[li].center - edge_pos).norm();
                    roadmap.addEdge(left_cells[li].center_id, edge_id, wL);
                    roadmap.addEdge(edge_id, left_cells[li].center_id, wL);

                    // Connect right center -> edge, edge -> right center
                    double wR = (right_cells[ri].center - edge_pos).norm();
                    roadmap.addEdge(right_cells[ri].center_id, edge_id, wR);
                    roadmap.addEdge(edge_id, right_cells[ri].center_id, wR);

                    // RViz marker for edge node (different color)
                    visualization_msgs::Marker edge_node;
                    edge_node.header.frame_id = "map";
                    edge_node.header.stamp = ros::Time::now();
                    edge_node.ns = "ecd_edge_nodes";
                    edge_node.id = marker_id++;
                    edge_node.type = visualization_msgs::Marker::SPHERE;
                    edge_node.pose.position.x = edge_pos.x();
                    edge_node.pose.position.y = edge_pos.y();
                    edge_node.pose.position.z = 0.06;
                    edge_node.scale.x = 0.07;
                    edge_node.scale.y = 0.07;
                    edge_node.scale.z = 0.07;
                    edge_node.color.r = 0.1;
                    edge_node.color.g = 0.9;
                    edge_node.color.b = 0.1;
                    edge_node.color.a = 1.0;
                    slab_markers.markers.push_back(edge_node);
                }
            }
        }
    }

    // ---- (Optional) connect vertical neighbors inside same slab (to allow vertical motion) - kept simple
    // for (int i = 0; i < N_SLICES; ++i) {
    //     for (size_t k = 0; k + 1 < slabs[i].size(); ++k) {
    //         int idA = slabs[i][k].center_id;
    //         int idB = slabs[i][k+1].center_id;
    //         Eigen::Vector2d pA = slabs[i][k].center;
    //         Eigen::Vector2d pB = slabs[i][k+1].center;
    //         double w = (pA - pB).norm();
    //         // Check midpoint collision before connecting
    //         Eigen::Vector2d mid = 0.5 * (pA + pB);
    //         if (!env.checkOccupancy(mid.x(), mid.y())) {
    //             roadmap.addEdge(idA, idB, w);
    //             roadmap.addEdge(idB, idA, w);
    //         }
    //     }
    // }

    // ---- Victim connection logic (keeps your existing pattern, using node_positions list)
    const int k_neighbors = 15;
    ROS_INFO("Connetting victims...");
    for (int i = 0; i < 2 + static_cast<int>(victims.size()); ++i) {
        std::vector<std::pair<double, int>> neighbors;

        // victims and special nodes are indices [0 .. 1+V], ECD nodes follow in node_positions
        for (int j = 2 + static_cast<int>(victims.size()); j < static_cast<int>(node_positions.size()); ++j) {
            if (i == j) continue;
            double d = (node_positions[i] - node_positions[j]).norm();
            neighbors.push_back({d, j});
        }
        std::sort(neighbors.begin(), neighbors.end());

        int connections_made = 0;
        for (size_t kk = 0; kk < neighbors.size() && connections_made < k_neighbors; ++kk) {
            int neighbor_idx = neighbors[kk].second;
            double distance = neighbors[kk].first;

            ROS_DEBUG("Checking collision between %d and %d", i, neighbor_idx);
            if (collisionFreeSegment(env, node_positions[i], node_positions[neighbor_idx])) {
                // map neighbor_idx (index in node_positions) back to roadmap node id:
                // we added nodes to roadmap in the same order as node_positions (start,goal,victims,centers,edges)
                // and we assigned explicit node ids in that same sequence, starting from 0 (start),1(goal),2.. victims, then centers/edges via next_id.
                // To get the roadmap node id for node_positions[k], we need to track the insertion mapping.
                // Simpler: iterate through roadmap.getNodes() to find the k-th inserted node id.
                int mapped_id = -1;
                int counter = 0;
                for (const auto& kv : roadmap.getNodes()) {
                    // Note: roadmap.getNodes() is std::map<int, Node>, iteration is by sorted key, not insertion order.
                    // To avoid mismatch, we can reconstruct a parallel vector of node ids in node_positions insertion order.
                    // But we kept the invariant: the sequence of node IDs we created was:
                    // start(0), goal(1), victims(2..), then centers/edges with increasing next_id.
                    // We can compute mapping directly:
                    if (counter == neighbor_idx) {
                        mapped_id = kv.first;
                        break;
                    }
                    ++counter;
                }
                // The above mapping using std::map iteration is not safe if keys are not ordered as insertion.
                // Instead, derive mapped id by formula when possible:
                // For indices < (2 + victims.size()) --> id == index
                int mapped_node_id = -1;
                if (neighbor_idx < 2 + static_cast<int>(victims.size())) {
                    mapped_node_id = neighbor_idx; // start/goal/victims ids are exactly indices
                } else {
                    // centers/edges were assigned consecutive ids starting at next_id_start
                    // We recorded next_id progression earlier: first center id was (2 + victims.size()), but since next_id was incremented while adding victims,
                    // the very first center had id = first_center_id = 2 + victims.size()
                    // And we appended node_positions accordingly, so node_positions index j corresponds to roadmap node id = j
                    mapped_node_id = neighbor_idx; // this works IF you keep node ids equal to node_positions index
                    // But in your current scheme we created node ids equal to consecutive integers starting at 0, so this identity holds.
                }
                if (mapped_node_id >= 0) {
                    roadmap.addEdge(mapped_node_id, mapped_node_id, 0.0); // noop to ensure map stays consistent (no-op, can be removed)
                    // proper addEdge victim_id <-> mapped_node_id:
                    int victim_id = i;
                    double d = distance;
                    roadmap.addEdge(victim_id, mapped_node_id, d);
                    // you may want add the reverse too:
                    roadmap.addEdge(mapped_node_id, victim_id, d);
                    ++connections_made;
                }
            }
        }
    }

    // publish markers
    slab_pub_.publish(slab_markers);
    ROS_INFO("Published %zu ECD slab markers", slab_markers.markers.size());

    ROS_INFO("ECD Roadmap built with %zu nodes", roadmap.getNodes().size());
    return roadmap;
}

bool collisionFreeSegment(
    const EnvironmentHandler& env,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    double step
) {
    double dist = (a - b).norm();
    if (dist < 0.001) return true;

    double res = 0.05;
    int steps = std::max(1, static_cast<int>(dist / res));
    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / static_cast<double>(steps);
        Eigen::Vector2d interpolated = a + ratio * (b - a);
        if (env.checkOccupancy(interpolated.x(), interpolated.y())) {
            return false;
        }
    }
    return true;
}

PLUGINLIB_EXPORT_CLASS(PlannerECD, PlannerBase)

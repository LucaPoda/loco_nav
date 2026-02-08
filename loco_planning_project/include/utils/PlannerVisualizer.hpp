#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "environment/Obstacle.hpp" 
#include "environment/Roadmap.hpp"
#include "orienteering/DistanceMatrix.hpp" // Ensure path is correct
#include "trajectory/DubinsTrajectory.hpp" // TODO: move TrajectoryPoint somewhere else and remove this import

class PlannerVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Publisher obstacles_pub_, borders_pub_, roadmap_pub_;
    
    // New Publishers
    ros::Publisher dist_matrix_pub_;    // All paths in the simplified roadmap
    ros::Publisher orienteering_pub_;   // Raw shortest path from OP solver
    ros::Publisher smoothed_pub_;       // Path after shortcutting
    ros::Publisher dubins_pub_;         // High-res Dubins trajectory

    // --- INTERNAL HELPERS ---

    // Helper to create a single path/line marker
    visualization_msgs::Marker createPathMarker(std::vector<geometry_msgs::Point>& points, 
                                              const std::string& ns, int id, 
                                              float r, float g, float b, float a,
                                              float width, float z_offset) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = width; 
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = a;
        marker.pose.orientation.w = 1.0;

        for (auto p : points) {
            p.z = z_offset; // Set layer
            marker.points.push_back(p);
        }
        return marker;
    }

    // Existing Obstacle/Border marker helper (Polygon/Circle)
    visualization_msgs::Marker createObsMarker(Obstacle& obs, int id, float r, float g, float b, std::string ns) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05; 
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;

        if (obs.getType() == Obstacle::POLYGON) {
            auto& verts = obs.getVertices(); 
            for (auto& v : verts) {
                geometry_msgs::Point p; p.x = v.x; p.y = v.y; p.z = 0.01;
                marker.points.push_back(p);
            }
            if (!verts.empty()) { 
                geometry_msgs::Point p; p.x = verts[0].x; p.y = verts[0].y; p.z = 0.01;
                marker.points.push_back(p);
            }
        } else {
            Point2D center = obs.getCentroid();
            double radius = obs.getInflatedRadius();
            for (int i = 0; i <= 40; ++i) {
                double angle = i * (2.0 * M_PI / 40);
                geometry_msgs::Point p;
                p.x = center.x + radius * cos(angle);
                p.y = center.y + radius * sin(angle);
                p.z = 0.01;
                marker.points.push_back(p);
            }
        }
        return marker;
    }

public:
    PlannerVisualizer() {
        obstacles_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("/planner/obstacles", 1, true);
        borders_pub_     = nh_.advertise<visualization_msgs::MarkerArray>("/planner/borders", 1, true);
        roadmap_pub_     = nh_.advertise<visualization_msgs::Marker>("/planner/roadmap", 1, true);
        
        dist_matrix_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/distance_matrix", 1, true);
        orienteering_pub_= nh_.advertise<visualization_msgs::Marker>("/planner/path_raw", 1, true);
        smoothed_pub_    = nh_.advertise<visualization_msgs::Marker>("/planner/path_smoothed", 1, true);
        dubins_pub_      = nh_.advertise<visualization_msgs::Marker>("/planner/trajectory_dubins", 1, true);
    }

    void publishDistanceMatrix(Roadmap& full_roadmap, DistanceMatrix& dist_matrix) {
        visualization_msgs::MarkerArray msg;
        auto& nodes = full_roadmap.getNodes();
        auto& cache = dist_matrix.getSearchCache();
        auto& interest_nodes = dist_matrix.getInterestNodes();
        
        int marker_id = 0;

        // Iterate through all unique pairs of interest nodes
        for (size_t i = 0; i < interest_nodes.size(); ++i) {
            for (size_t j = i + 1; j < interest_nodes.size(); ++j) {
                int u = interest_nodes[i];
                int v = interest_nodes[j];

                // Use the DijkstraResult helper to get the sequence of node IDs
                if (cache.count(u)) {
                    std::vector<int> node_ids = cache.at(u).getPathTo(v);
                    
                    if (node_ids.empty()) continue;

                    // Convert node IDs to geometry_msgs::Point
                    std::vector<geometry_msgs::Point> points;
                    for (int id : node_ids) {
                        if (nodes.count(id)) {
                            geometry_msgs::Point p;
                            p.x = nodes.at(id).position.x();
                            p.y = nodes.at(id).position.y();
                            // Layer this at the bottom of our path stack
                            points.push_back(p);
                        }
                    }

                    // Publish as a thin, semi-transparent grey line strip
                    msg.markers.push_back(createPathMarker(
                        points, 
                        "distance_matrix_paths", 
                        marker_id++, 
                        0.6, 0.6, 0.6, 0.6, // R, G, B, A (Light Grey, very transparent)
                        0.01,               // Width (Thin)
                        0.05                // Z-offset (Bottom layer)
                    ));
                }
            }
        }
        dist_matrix_pub_.publish(msg);
    }

    // 2. The final shortest path after orienteering (Raw Node sequence)
    void publishOrienteeringPath(Roadmap& roadmap, std::vector<int>& path_ids) {
        std::vector<geometry_msgs::Point> points;
        auto& nodes = roadmap.getNodes();
        for (int id : path_ids) {
            geometry_msgs::Point p;
            p.x = nodes.at(id).position.x(); p.y = nodes.at(id).position.y();
            points.push_back(p);
        }
        // Color: Orange, Width: Medium, Z: 0.1
        orienteering_pub_.publish(createPathMarker(points, "raw_path", 0.5, 1.0, 0.0, 0.0, 0.8, 0.04, 0.1));
    }

    // 3. The smoothed path (Shortcut nodes)
    void publishSmoothedPath(Roadmap& roadmap, std::vector<int>& path_ids) {
        std::vector<geometry_msgs::Point> points;
        auto& nodes = roadmap.getNodes();
        for (int id : path_ids) {
            geometry_msgs::Point p;
            p.x = nodes.at(id).position.x(); p.y = nodes.at(id).position.y();
            points.push_back(p);
        }
        // Color: Green, Width: Thick, Z: 0.15
        smoothed_pub_.publish(createPathMarker(points, "smoothed", 0, 0.0, 1.0, 0.0, 1.0, 0.06, 0.15));
    }

    // 4. The high-res Dubins trajectory
    void publishDubinsTrajectory(std::vector<TrajectoryPoint>& trajectory) {
        std::vector<geometry_msgs::Point> points;
        for (auto& tp : trajectory) {
            geometry_msgs::Point p; p.x = tp.x; p.y = tp.y;
            points.push_back(p);
        }
        // Color: Blue, Width: Thin (but solid), Z: 0.2
        dubins_pub_.publish(createPathMarker(points, "dubins", 0, 0.0, 0.0, 1.0, 1.0, 0.03, 0.2));
    }
    

    void publishObstacles(std::vector<Obstacle>& obstacles) {
        visualization_msgs::MarkerArray msg;
        int id = 0;
        for (auto& obs : obstacles) {
            msg.markers.push_back(createObsMarker(obs, id++, 1.0, 0.0, 0.0, "obstacles"));
        }
        obstacles_pub_.publish(msg);
    }


    void publishBorders(Obstacle& border) {
        visualization_msgs::MarkerArray msg;
        msg.markers.push_back(createObsMarker(border, 0, 0.0, 0.0, 1.0, "border"));
        borders_pub_.publish(msg);
    }


    void publishRoadmap(Roadmap& roadmap) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "roadmap";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = 0.7; marker.color.g = 0.7; marker.color.b = 0.7; marker.color.a = 0.2; // light gray
        marker.pose.orientation.w = 1.0;

        auto& nodes = roadmap.getNodes();
        auto edges = roadmap.getEdges();

        for (auto& edge : edges) {
            if (nodes.count(edge.u) && nodes.count(edge.v)) {
                geometry_msgs::Point p1, p2;
                p1.x = nodes.at(edge.u).position.x();
                p1.y = nodes.at(edge.u).position.y();
                p1.z = 0.02;
                p2.x = nodes.at(edge.v).position.x();
                p2.y = nodes.at(edge.v).position.y();
                p2.z = 0.02;
                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }
        roadmap_pub_.publish(marker);
    }

    

}; // THE IMPORTANT SEMICOLON

#endif
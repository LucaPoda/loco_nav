#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "environemnt/Obstacles.hpp" 
#include "environemnt/Roadmap.hpp"

class PlannerVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Publisher obstacles_pub_;
    ros::Publisher borders_pub_;
    ros::Publisher roadmap_pub_;
    //ros::Publisher special_paths_pub_;

    // Helper method MUST be defined before it is used, or declared here
    visualization_msgs::Marker createMarker(const Obstacle& obs, int id, float r, float g, float b, std::string ns) {
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
        marker.lifetime = ros::Duration(0); 

        if (obs.getType() == Obstacle::POLYGON) {
            const auto& verts = obs.getVertices(); 
            for (const auto& v : verts) {
                geometry_msgs::Point p; p.x = v.x; p.y = v.y; p.z = 0.0;
                marker.points.push_back(p);
            }
            if (!verts.empty()) { // Close loop
                geometry_msgs::Point p; p.x = verts[0].x; p.y = verts[0].y; p.z = 0.0;
                marker.points.push_back(p);
            }
        } 
        else {
            Point2D center = obs.getCentroid();
            double radius = obs.getInflatedRadius();
            int segments = 40;
            for (int i = 0; i <= segments; ++i) {
                double angle = i * (2.0 * M_PI / segments);
                geometry_msgs::Point p;
                p.x = center.x + radius * cos(angle);
                p.y = center.y + radius * sin(angle);
                p.z = 0.0;
                marker.points.push_back(p);
            }
        }
        return marker;
    }

public:
    PlannerVisualizer() {
        obstacles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/obstacles", 1, true);
        borders_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("/planner/borders", 1, true);
        roadmap_pub_   = nh_.advertise<visualization_msgs::Marker>("/planner/roadmap", 1, true);
        //special_paths_pub_   = nh_.advertise<visualization_msgs::Marker>("/planner/special_paths", 1, true);
    } // Fixed constructor semicolon issue

    void publishObstacles(const std::vector<Obstacle>& obstacles) {
        visualization_msgs::MarkerArray msg;
        int id = 0;
        for (const auto& obs : obstacles) {
            msg.markers.push_back(createMarker(obs, id++, 1.0, 0.0, 0.0, "obstacles"));
        }
        obstacles_pub_.publish(msg);
    }

    void publishBorders(const Obstacle& border) {
        visualization_msgs::MarkerArray msg;
        msg.markers.push_back(createMarker(border, 0, 0.0, 0.0, 1.0, "border"));
        borders_pub_.publish(msg);
    }

    void publishRoadmap(const Roadmap& roadmap) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "roadmap";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.6;
        marker.pose.orientation.w = 1.0;

        const auto& nodes = roadmap.getNodes();
        const auto edges = roadmap.getEdges();

        for (const auto& edge : edges) {
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

    // void publishSpecialPaths(const std::map<int, std::map<int, double>>& cost_matrix, const Roadmap& roadmap) {
    //     visualization_msgs::MarkerArray msg;
    //     int marker_id = 0;

    //     const auto& nodes = roadmap.getNodes();

    //     for (auto const& [source, targets] : cost_matrix) {
    //         for (auto const& [target, dist] : targets) {
    //             // Only draw if a path exists and we haven't drawn the reverse already
    //             if (dist < std::numeric_limits<double>::infinity() && source < target) {
                    
    //                 visualization_msgs::Marker path_marker;
    //                 path_marker.header.frame_id = "map";
    //                 path_marker.header.stamp = ros::Time::now();
    //                 path_marker.ns = "special_links";
    //                 path_marker.id = marker_id++;
    //                 path_marker.type = visualization_msgs::Marker::LINE_LIST;
    //                 path_marker.scale.x = 0.04; // Medium thickness
    //                 path_marker.color.r = 0.6; path_marker.color.g = 0.0; path_marker.color.b = 1.0; // Purple
    //                 path_marker.color.a = 0.8;

    //                 // For the "high-level" view, we can just draw straight lines between 
    //                 // the special nodes, OR call findShortestPath to get the actual curves.
    //                 // Let's draw straight lines for the "Abstract Graph" view:
    //                 geometry_msgs::Point p1, p2;
    //                 p1.x = nodes.at(source).position.x();
    //                 p1.y = nodes.at(source).position.y();
    //                 p1.z = 0.1; // Float above roadmap
                    
    //                 p2.x = nodes.at(target).position.x();
    //                 p2.y = nodes.at(target).position.y();
    //                 p2.z = 0.1;

    //                 path_marker.points.push_back(p1);
    //                 path_marker.points.push_back(p2);
    //                 msg.markers.push_back(path_marker);
    //             }
    //         }
    //     }
    //     special_paths_pub_.publish(msg);
    // }
}; // THE IMPORTANT SEMICOLON

#endif
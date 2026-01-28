#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "utils/obstacles.hpp" 

class PlannerVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Publisher obstacles_pub_;
    ros::Publisher borders_pub_;

public:
    PlannerVisualizer() {
        // 1. Obstacles Publisher (Dynamic)
        // Not latched (latch=false), because these update constantly.
        obstacles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/obstacles", 1);

        // 2. Borders Publisher (Static)
        // LATCHED (latch=true). This means we publish once, and ROS keeps 
        // sending it to any new node (like RViz) that joins later.
        borders_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/borders", 1, true);
    }

    // Call this inside your loop (Frequent updates)
    void publishObstacles(const std::vector<Obstacle>& obstacles) {
        visualization_msgs::MarkerArray msg;
        int id = 0;
        for (const auto& obs : obstacles) {
            // RED Color for Obstacles
            msg.markers.push_back(createMarker(obs, id++, 1.0, 0.0, 0.0, "obstacles"));
        }
        obstacles_pub_.publish(msg);
    }

    // Call this ONLY when you receive the map/borders (Rare updates)
    void publishBorders(const Obstacle& border) {
        visualization_msgs::MarkerArray msg;
        // BLUE Color for Borders
        msg.markers.push_back(createMarker(border, 0, 0.0, 0.0, 1.0, "border"));
        borders_pub_.publish(msg);
    }

private:
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
                geometry_msgs::Point p; p.x = v.x; p.y = v.y; marker.points.push_back(p);
            }
            if (!verts.empty()) { // Close loop
                geometry_msgs::Point p; p.x = verts[0].x; p.y = verts[0].y; marker.points.push_back(p);
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
                marker.points.push_back(p);
            }
        }
        return marker;
    }
};
#endif
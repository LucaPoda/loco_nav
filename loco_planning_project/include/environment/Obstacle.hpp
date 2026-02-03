#pragma once

#include <geometry_msgs/Polygon.h>
#include <obstacles_msgs/ObstacleMsg.h>
#include <Eigen/Dense>
#include <vector>

struct Point2D {
    double x, y;
};

class Obstacle {
public:
    enum Type { CIRCLE, POLYGON };

    // Constructors
    Obstacle();
    Obstacle(const geometry_msgs::Polygon& poly);
    Obstacle(const obstacles_msgs::ObstacleMsg& msg);

    // Core Logic
    void computeCSpace(double offset_dist, double arc_tolerance = 0.01);
    bool checkCollision(const Eigen::Vector3d& p) const;
    bool checkCollision(double x, double y) const;
    bool isInsideAABB(double x, double y) const;

    // Getters
    double getMinX() const { return aabb_min_.x; }
    double getMaxX() const { return aabb_max_.x; }
    double getMinY() const { return aabb_min_.y; }
    double getMaxY() const { return aabb_max_.y; }
    
    const std::vector<Point2D>& getVertices() const { return c_space_polygon_; }
    Type getType() const { return type_; }
    Point2D getCentroid() const { return centroid_; }
    double getInflatedRadius() const { return inflated_radius_; }

private:
    Type type_;
    std::vector<Point2D> raw_polygon_;
    std::vector<Point2D> c_space_polygon_;
    Point2D centroid_;
    double radius_;
    double inflated_radius_;
    Point2D aabb_min_;
    Point2D aabb_max_;

    int inflate_direction_ = 1; // Inflate outward by default
};
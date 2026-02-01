#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <geometry_msgs/Polygon.h>
#include <obstacles_msgs/ObstacleMsg.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "clipper2/clipper.h" 

struct Point2D {
    double x, y;
};

class Obstacle {
public:
    enum Type { CIRCLE, POLYGON };

    // Default Constructor (Required for vectors)
    Obstacle() : type_(CIRCLE), radius_(0.0), inflated_radius_(0.0) {
        centroid_ = {0.0, 0.0};
    }

    // BORDER CONSTRUCTOR: from simple polygon message
    Obstacle(const geometry_msgs::Polygon& poly) : type_(POLYGON), radius_(0.0), inflated_radius_(0.0) {
        for (const auto& p : poly.points) {
            raw_polygon_.push_back({p.x, p.y});
        }
        inflate_direction_ = -1; // borders should be inflated inward
    }

    // OBSTACLE CONSTRUCTOR: from obstacle message
    Obstacle(const obstacles_msgs::ObstacleMsg& msg) {
        if (msg.polygon.points.size() == 1) {
            type_ = CIRCLE;
            centroid_ = {msg.polygon.points[0].x, msg.polygon.points[0].y};
            radius_ = msg.radius;
        } else {
            type_ = POLYGON;
            for (const auto& p : msg.polygon.points) {
                raw_polygon_.push_back({p.x, p.y});
            }
        }
        inflate_direction_ = 1; // borders should be inflated outward
    }

    void computeCSpace(double offset_dist, double arc_tolerance = 0.01) {
        c_space_polygon_.clear();
        aabb_min_ = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        aabb_max_ = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

        if (type_ == CIRCLE) {
            inflated_radius_ = radius_ + offset_dist;
            aabb_min_ = {centroid_.x - inflated_radius_, centroid_.y - inflated_radius_};
            aabb_max_ = {centroid_.x + inflated_radius_, centroid_.y + inflated_radius_};
        } 
        else {
            Clipper2Lib::Paths64 subject;
            Clipper2Lib::Path64 p;
            const double scale = 1e6; // scale dimensions up by 1e6 for increasing precision in floating point algebraic math computation
            for (const auto& pt : raw_polygon_) {
                p.push_back(Clipper2Lib::Point64(pt.x * scale, pt.y * scale));
            }
            subject.push_back(p);

            Clipper2Lib::Paths64 solution = Clipper2Lib::InflatePaths(
                subject, 
                offset_dist * scale * inflate_direction_,  
                Clipper2Lib::JoinType::Round, 
                Clipper2Lib::EndType::Polygon, 
                2.0, 
                arc_tolerance * scale
            );

            if (!solution.empty()) { // scale dimensions back and create che c_space polygon
                for (const auto& pt : solution[0]) {
                    double px = (double)pt.x / scale;
                    double py = (double)pt.y / scale;
                    c_space_polygon_.push_back({px, py});

                    // store a simple bounding box for fast collision computation
                    if (px < aabb_min_.x) aabb_min_.x = px;
                    if (py < aabb_min_.y) aabb_min_.y = py;
                    if (px > aabb_max_.x) aabb_max_.x = px;
                    if (py > aabb_max_.y) aabb_max_.y = py;
                }
            }
        }
    }

    bool isInsideAABB(double x, double y) const {
        return (x >= aabb_min_.x && x <= aabb_max_.x &&
                y >= aabb_min_.y && y <= aabb_max_.y);
    }

    bool checkCollision(const Eigen::Vector3d& p) const {
        return checkCollision(p.x(), p.y());
    }

    // checks collisions with the specified point
    bool checkCollision(double x, double y) const {
        // check the simple bounding box to discard points that are definitely not inside. Only when inflate_direction is > 0: obstacles and not borders 
        if (!isInsideAABB(x, y) && inflate_direction_ > 0) return false;
        
        // TODO: only false negative for obstacles contact are discarded, we should implement a similar logic also for borders, 
        // computing an inscribed bounding box or an inscribed circle

        bool inside = false;
        if (type_ == CIRCLE) {
            inside = std::hypot(x - centroid_.x, y - centroid_.y) <= inflated_radius_;
        } 
        else {
            // Ray-Casting (Jordan Curve Theorem): cast a ray in an arbitrary direction and count the number of intersected edges:
            // - even number: outside -> inside = false (no-collision)
            // - odd number: inside -> inside = true (collision happened)

            size_t n = c_space_polygon_.size();
            for (size_t i = 0, j = n - 1; i < n; j = i++) { // check pairs of vertices
                double xi = c_space_polygon_[i].x;
                double yi = c_space_polygon_[i].y;

                double xj = c_space_polygon_[j].x;
                double yj = c_space_polygon_[j].y;

                bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

                if (intersect) inside = !inside;
            }
        }

        // return collision based on the type of obstacle (map border / obstacle)
        return (inside && inflate_direction_ > 0) || // collision with obstacles (inside the inflated polygon)
            (!inside && inflate_direction_ < 0); // collision with borders (outside the inflated polygon)
    }

    // Needed to get bbox of the 
    double getMinX() const { return aabb_min_.x; }
    double getMaxX() const { return aabb_max_.x; }
    double getMinY() const { return aabb_min_.y; }
    double getMaxY() const { return aabb_max_.y; }
    
    // Getters
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
#endif
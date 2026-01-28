#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <geometry_msgs/Polygon.h>
#include <obstacles_msgs/ObstacleMsg.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include <utils/dubins_ompl.hpp>

// Include Clipper2 (Header-only or library)
#include "clipper2/clipper.h" 

struct Point2D {
    double x, y;
};

class Obstacle {
public:
    enum Type { CIRCLE, POLYGON };

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
            // Ensure polygon is closed for Clipper? 
            // Clipper handles open paths, but for inflation we treat it as polygon.
        }
    }

    /**
     * @brief Computes the Configuration Space Obstacle (Detailed Polygon)
     * @param offset_dist Robot Radius + Safety Margin
     * @param arc_tolerance Max error allowed for arc approximation (default 1cm)
     */
    void computeCSpace(double offset_dist, double arc_tolerance = 0.01) {
        c_space_polygon_.clear();
        aabb_min_ = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        aabb_max_ = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

        if (type_ == CIRCLE) {
            // For a circle obstacle, C-Space is just a bigger circle
            inflated_radius_ = radius_ + offset_dist;
            
            // Update AABB for the circle
            aabb_min_ = {centroid_.x - inflated_radius_, centroid_.y - inflated_radius_};
            aabb_max_ = {centroid_.x + inflated_radius_, centroid_.y + inflated_radius_};
        } 
        else {
            // --- CLIPPER2 PRECISE OFFSETTING ---
            Clipper2Lib::Paths64 subject;
            Clipper2Lib::Path64 p;
            
            const double scale = 1e6; // Scale to preserve precision in integer math
            for (const auto& pt : raw_polygon_) {
                p.push_back(Clipper2Lib::Point64(pt.x * scale, pt.y * scale));
            }
            subject.push_back(p);

            // The "Right Way": Use JoinType::Round
            // This generates vertices along the arc to keep error < arc_tolerance
            Clipper2Lib::Paths64 solution = Clipper2Lib::InflatePaths(
                subject, 
                offset_dist * scale, 
                Clipper2Lib::JoinType::Round, 
                Clipper2Lib::EndType::Polygon, 
                2.0, // Miter limit (unused for Round)
                arc_tolerance * scale // Precision control
            );

            if (!solution.empty()) {
                // Store result and Compute AABB simultaneously
                for (const auto& pt : solution[0]) {
                    double px = (double)pt.x / scale;
                    double py = (double)pt.y / scale;
                    c_space_polygon_.push_back({px, py});

                    if (px < aabb_min_.x) aabb_min_.x = px;
                    if (py < aabb_min_.y) aabb_min_.y = py;
                    if (px > aabb_max_.x) aabb_max_.x = px;
                    if (py > aabb_max_.y) aabb_max_.y = py;
                }
            }
        }
    }

    // --- COLLISION DETECTION ---
    
    // Fast check: Is point inside the bounding box?
    bool isInsideAABB(double x, double y) const {
        return (x >= aabb_min_.x && x <= aabb_max_.x &&
                y >= aabb_min_.y && y <= aabb_max_.y);
    }

    // Precise check: Is point inside the actual shape?
    bool checkCollision(double x, double y) const {
        // 1. Broad Phase: AABB Check
        if (!isInsideAABB(x, y)) return false;

        // 2. Narrow Phase
        if (type_ == CIRCLE) {
            return std::hypot(x - centroid_.x, y - centroid_.y) <= inflated_radius_;
        } else {
            // Standard Ray Casting / Winding Number for Polygon
            // Works for any polygon (convex or concave)
            bool inside = false;
            size_t n = c_space_polygon_.size();
            for (size_t i = 0, j = n - 1; i < n; j = i++) {
                double xi = c_space_polygon_[i].x, yi = c_space_polygon_[i].y;
                double xj = c_space_polygon_[j].x, yj = c_space_polygon_[j].y;

                bool intersect = ((yi > y) != (yj > y)) &&
                                 (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
                if (intersect) inside = !inside;
            }
            return inside;
        }
    }

    // Getters for Visibility Graph
    const std::vector<Point2D>& getVertices() const { return c_space_polygon_; }
    Type getType() const { return type_; }
    
    // For Circle types, we might want to discretize it to add to Visibility Graph
    std::vector<Point2D> getCircleDiscretization(int resolution=16) const {
        std::vector<Point2D> pts;
        if (type_ != CIRCLE) return pts;
        
        for(int i=0; i<resolution; ++i) {
            double theta = 2.0 * M_PI * i / resolution;
            pts.push_back({
                centroid_.x + inflated_radius_ * cos(theta),
                centroid_.y + inflated_radius_ * sin(theta)
            });
        }
        return pts;
    }

private:
    Type type_;
    std::vector<Point2D> raw_polygon_;
    std::vector<Point2D> c_space_polygon_; // The inflated result
    
    Point2D centroid_;
    double radius_;
    double inflated_radius_;

    // Axis Aligned Bounding Box (AABB) for optimization
    Point2D aabb_min_;
    Point2D aabb_max_;
};

#endif
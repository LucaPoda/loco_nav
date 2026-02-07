#include "environment/Obstacle.hpp"
#include "clipper2/clipper.h" 
#include <cmath>
#include <limits>
#include <algorithm>

Obstacle::Obstacle() : type_(CIRCLE), radius_(0.0), inflated_radius_(0.0) {
    centroid_ = {0.0, 0.0};
}

Obstacle::Obstacle(const geometry_msgs::Polygon& poly) : type_(POLYGON), radius_(0.0), inflated_radius_(0.0) {
    for (const auto& p : poly.points) {
        raw_polygon_.push_back({p.x, p.y});
    }
    inflate_direction_ = -1; // borders should be inflated inward
}

Obstacle::Obstacle(const obstacles_msgs::ObstacleMsg& msg) {
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
    inflate_direction_ = 1; 
}

void Obstacle::computeCSpace(double offset_dist, double arc_tolerance) {
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
        const double scale = 1e6; 
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

        if (!solution.empty()) {
            for (const auto& pt : solution[0]) {
                double px = static_cast<double>(pt.x) / scale;
                double py = static_cast<double>(pt.y) / scale;
                c_space_polygon_.push_back({px, py});

                if (px < aabb_min_.x) aabb_min_.x = px;
                if (py < aabb_min_.y) aabb_min_.y = py;
                if (px > aabb_max_.x) aabb_max_.x = px;
                if (py > aabb_max_.y) aabb_max_.y = py;
            }
        }
    }
}

bool Obstacle::isInsideAABB(double x, double y) const {
    return (x >= aabb_min_.x && x <= aabb_max_.x &&
            y >= aabb_min_.y && y <= aabb_max_.y);
}

bool Obstacle::checkCollision(const Eigen::Vector3d& p) const {
    return checkCollision(p.x(), p.y());
}

bool Obstacle::checkCollision(double x, double y) const {
    // AABB Check (for efficiency)
    if (!isInsideAABB(x, y) && inflate_direction_ > 0) return false;
    
    bool inside = false;
    if (type_ == CIRCLE) {
        inside = std::hypot(x - centroid_.x, y - centroid_.y) <= inflated_radius_;
    } 
    else {
        // Ray-Casting Algorithm
        size_t n = c_space_polygon_.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = c_space_polygon_[i].x;
            double yi = c_space_polygon_[i].y;
            double xj = c_space_polygon_[j].x;
            double yj = c_space_polygon_[j].y;

            bool intersect = ((yi > y) != (yj > y)) && 
                             (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
        }
    }

    return (inside && inflate_direction_ > 0) || (!inside && inflate_direction_ < 0);
}
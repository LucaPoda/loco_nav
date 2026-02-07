#include "planners/planner_acd.hpp"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <limits>
#include <algorithm>
#include <queue>

PlannerACD::PlannerACD() {}
PlannerACD::~PlannerACD() {}

void PlannerACD::initialize(const std::string& robot_name) {
    PlannerBase::initialize(robot_name);

    ros::NodeHandle nh;
    slab_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/acd/quadtree", 1, true);

    ros::NodeHandle pnh("~");
    pnh.param("quadtree_max_depth", quadtree_max_depth_, 6);
    pnh.param("quadtree_min_size", quadtree_min_size_, 0.25);
    pnh.param("k_victim_neighbors", k_victim_neighbors_, 15);

    ROS_INFO("Initializing ACD (Quadtree) Planner plugin...");
    ROS_INFO("  quadtree_max_depth: %d", quadtree_max_depth_);
    ROS_INFO("  quadtree_min_size:  %.3f", quadtree_min_size_);
    ROS_INFO("  k_victim_neighbors: %d", k_victim_neighbors_);
}

static bool collisionFreeSegment(const EnvironmentHandler& env, const Eigen::Vector2d& a, const Eigen::Vector2d& b, double step = 0.05) {
    double dist = (a - b).norm();
    if (dist < 1e-9) return true;
    int steps = std::max(1, static_cast<int>(dist / step));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector2d p = a + t * (b - a);
        if (env.checkOccupancy(p.x(), p.y())) return false;
    }
    return true;
}

// geometry helpers
struct Quad { double xl, xr, yl, yr; int depth; };

static bool quadHasOccupiedCorner(const EnvironmentHandler& env, const Quad& q) {
    return env.checkOccupancy(q.xl, q.yl) ||
           env.checkOccupancy(q.xl, q.yr) ||
           env.checkOccupancy(q.xr, q.yl) ||
           env.checkOccupancy(q.xr, q.yr);
}

// return true if quad extends outside map bbox
static bool quadOutsideBounds(const Quad& q, double min_x, double max_x, double min_y, double max_y) {
    const double EPS = 1e-9;
    return (q.xl < min_x - EPS) || (q.xr > max_x + EPS) || (q.yl < min_y - EPS) || (q.yr > max_y + EPS);
}

// orientation + segment intersection (standard robust-ish predicates)
static int orient(double ax, double ay, double bx, double by, double cx, double cy) {
    double v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
}
static bool onSegment(double ax, double ay, double bx, double by, double px, double py) {
    return std::min(ax,bx) <= px + 1e-9 && px <= std::max(ax,bx) + 1e-9 &&
           std::min(ay,by) <= py + 1e-9 && py <= std::max(ay,by) + 1e-9;
}
static bool segIntersectsSeg(double a_x, double a_y, double b_x, double b_y,
                             double c_x, double c_y, double d_x, double d_y) {
    int o1 = orient(a_x,a_y, b_x,b_y, c_x,c_y);
    int o2 = orient(a_x,a_y, b_x,b_y, d_x,d_y);
    int o3 = orient(c_x,c_y, d_x,d_y, a_x,a_y);
    int o4 = orient(c_x,c_y, d_x,d_y, b_x,b_y);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(a_x,a_y, b_x,b_y, c_x,c_y)) return true;
    if (o2 == 0 && onSegment(a_x,a_y, b_x,b_y, d_x,d_y)) return true;
    if (o3 == 0 && onSegment(c_x,c_y, d_x,d_y, a_x,a_y)) return true;
    if (o4 == 0 && onSegment(c_x,c_y, d_x,d_y, b_x,b_y)) return true;
    return false;
}

// check rectangle circle intersection
static bool rectIntersectsCircle(const Quad& q, const Eigen::Vector2d& center, double radius) {
    // find closest point to circle center on rect
    double cx = std::max(q.xl, std::min(center.x(), q.xr));
    double cy = std::max(q.yl, std::min(center.y(), q.yr));
    double dx = center.x() - cx;
    double dy = center.y() - cy;
    return (dx*dx + dy*dy) <= (radius * radius + 1e-9);
}

// polygon-rectangle intersection using vertex-in-rect, rect-corner-in-poly, and edge-edge intersection
static bool polygonIntersectsRect(const Obstacle& obs, const Quad& q) {
    const auto& verts = obs.getVertices();
    if (verts.empty()) {
        // fallback: treat via centroid+inflated radius
        Eigen::Vector2d c(obs.getCentroid().x, obs.getCentroid().y);
        double r = obs.getInflatedRadius();
        return rectIntersectsCircle(q, c, r);
    }

    // 1) any polygon vertex inside rect?
    for (const auto& v : verts) {
        if (v.x >= q.xl - 1e-9 && v.x <= q.xr + 1e-9 && v.y >= q.yl - 1e-9 && v.y <= q.yr + 1e-9) {
            return true;
        }
    }

    // 2) any rect corner inside polygon?
    double corners[4][2] = {
        {q.xl, q.yl},
        {q.xl, q.yr},
        {q.xr, q.yl},
        {q.xr, q.yr}
    };
    for (int k = 0; k < 4; ++k) {
        if (obs.checkCollision(corners[k][0], corners[k][1])) return true;
    }

    // 3) any polygon edge intersects any rect edge?
    auto rectEdges = std::array<std::pair<Eigen::Vector2d,Eigen::Vector2d>,4>{
        std::make_pair(Eigen::Vector2d(q.xl,q.yl), Eigen::Vector2d(q.xr,q.yl)),
        std::make_pair(Eigen::Vector2d(q.xr,q.yl), Eigen::Vector2d(q.xr,q.yr)),
        std::make_pair(Eigen::Vector2d(q.xr,q.yr), Eigen::Vector2d(q.xl,q.yr)),
        std::make_pair(Eigen::Vector2d(q.xl,q.yr), Eigen::Vector2d(q.xl,q.yl))
    };

    for (size_t i = 0; i + 1 < verts.size(); ++i) {
        const auto& a = verts[i];
        const auto& b = verts[i+1];
        for (const auto& re : rectEdges) {
            if (segIntersectsSeg(a.x, a.y, b.x, b.y,
                                 re.first.x(), re.first.y(),
                                 re.second.x(), re.second.y())) {
                return true;
            }
        }
    }
    // last->first
    if (verts.size() >= 2) {
        const auto& a = verts.back();
        const auto& b = verts.front();
        for (const auto& re : rectEdges) {
            if (segIntersectsSeg(a.x, a.y, b.x, b.y,
                                 re.first.x(), re.first.y(),
                                 re.second.x(), re.second.y())) {
                return true;
            }
        }
    }

    return false;
}

// Test whether obstacle intersects the quad (geometry-based)
static bool obstacleIntersectsQuad(const Obstacle& obs, const Quad& q) {
    if (obs.getType() == Obstacle::CIRCLE) {
        Eigen::Vector2d c(obs.getCentroid().x, obs.getCentroid().y);
        double r = obs.getInflatedRadius();
        return rectIntersectsCircle(q, c, r);
    } else {
        return polygonIntersectsRect(obs, q);
    }
}

static bool quadFullyInsideObstacle(const Obstacle& obs, const Quad& q) {
    double corners[4][2] = {
        {q.xl, q.yl},
        {q.xl, q.yr},
        {q.xr, q.yl},
        {q.xr, q.yr}
    };
    for (int k=0;k<4;++k) {
        if (!obs.checkCollision(corners[k][0], corners[k][1])) {
            return false;
        }
    }
    return true;
}

static bool quadsAreAdjacent(const Quad& a, const Quad& b) {
    const double EPS = 1e-9;
    if (std::abs(a.xr - b.xl) < EPS || std::abs(b.xr - a.xl) < EPS) {
        double lo = std::max(a.yl, b.yl);
        double hi = std::min(a.yr, b.yr);
        return hi - lo > EPS;
    }
    if (std::abs(a.yr - b.yl) < EPS || std::abs(b.yr - a.yl) < EPS) {
        double lo = std::max(a.xl, b.xl);
        double hi = std::min(a.xr, b.xr);
        return hi - lo > EPS;
    }
    return false;
}

Roadmap PlannerACD::buildRoadmap() {
    Roadmap roadmap;
    const auto& env = getEnvironment();
    const auto& params = getParams();

    ROS_INFO("PlannerACD: building roadmap...");

    // Add special nodes to roadmap
    Eigen::Vector2d start = env.getStartPose().head<2>();
    Eigen::Vector2d goal  = env.getGoalPose().head<2>();
    roadmap.addNode(0, start, 0.0);
    roadmap.addNode(1, goal,  0.0);
    
    // Add victims to roadmap
    const auto& victims = env.getVictims();
    int next_id = 2;
    for (size_t i = 0; i < victims.size(); ++i) {
        int vid = next_id++;
        roadmap.addNode(vid, victims[i].position, victims[i].reward);
    }

    // Build tree root
    double min_x = env.getMinX();
    double max_x = env.getMaxX();
    double min_y = env.getMinY();
    double max_y = env.getMaxY();
    if (max_x <= min_x) { max_x = min_x + 1.0; }
    if (max_y <= min_y) { max_y = min_y + 1.0; }
    double w = max_x - min_x;
    double h = max_y - min_y;
    double half = 0.5 * std::max(w, h);
    double cx = 0.5 * (min_x + max_x);
    double cy = 0.5 * (min_y + max_y);

    Quad root{cx - half, cx + half, cy - half, cy + half, 0};

    // subdivision
    std::vector<Quad> leaves;
    std::vector<Quad> to_process;
    to_process.push_back(root);

    const auto& obstacles = env.getObstacles();

    while (!to_process.empty()) {
        Quad q = to_process.back();
        to_process.pop_back();

        double q_w = q.xr - q.xl;
        double q_h = q.yr - q.yl;
        double size_min = std::min(q_w, q_h);

        bool outside = quadOutsideBounds(q, min_x, max_x, min_y, max_y);

        // Check if the quad exits the map
        bool has_occupied_corner = quadHasOccupiedCorner(env, q);

        // Check intersections with obstacles
        bool intersects_any = false;
        bool fully_covered = false;
        for (const auto& obs : obstacles) {
            if (obstacleIntersectsQuad(obs, q)) {
                intersects_any = true;
            }
            if (quadFullyInsideObstacle(obs, q)) {
                fully_covered = true;
                break;
            }
        }

        // Decide splitting
        bool should_split = (q.depth < quadtree_max_depth_ && size_min > quadtree_min_size_) && (outside || has_occupied_corner || intersects_any);

        // always split a little, to add more control points
        if (q.depth < 2) should_split = true;

        if (!should_split) {
            // Accept as leaf
            if (!outside && !has_occupied_corner && !fully_covered && !intersects_any) {
                leaves.push_back(q);
            }
            // Go to the next one
            continue;
        }

        // split
        double xm = 0.5*(q.xl + q.xr);
        double ym = 0.5*(q.yl + q.yr);
        Quad q1{q.xl, xm, q.yl, ym, q.depth + 1};
        Quad q2{xm, q.xr, q.yl, ym, q.depth + 1};
        Quad q3{q.xl, xm, ym, q.yr, q.depth + 1};
        Quad q4{xm, q.xr, ym, q.yr, q.depth + 1};
        to_process.push_back(q1);
        to_process.push_back(q2);
        to_process.push_back(q3);
        to_process.push_back(q4);
    }

    ROS_INFO("Quadtree (geom) produced %zu leaf cells", leaves.size());
    if (leaves.empty()) {
        ROS_ERROR("Quadtree produced zero free leaves. Aborting roadmap construction.");
        visualization_msgs::MarkerArray empty_ma;
        slab_pub_.publish(empty_ma);
        return roadmap;
    }

    // Create a node per leaf
    struct CellInfo { Quad q; int center_id; Eigen::Vector2d center; };
    std::vector<CellInfo> cells;
    cells.reserve(leaves.size());

    for (const auto& q : leaves) {
        Eigen::Vector2d center(0.5*(q.xl + q.xr), 0.5*(q.yl + q.yr));
        int cid = next_id++;
        roadmap.addNode(cid, center, 0.0);
        CellInfo ci; ci.q = q; ci.center_id = cid; ci.center = center;
        cells.push_back(ci);
    }

    // Connect adjacent cells if the path is clear
    for (size_t i = 0; i < cells.size(); ++i) {
        for (size_t j = i + 1; j < cells.size(); ++j) {
            if (!quadsAreAdjacent(cells[i].q, cells[j].q)) continue;
            Eigen::Vector2d mid = 0.5 * (cells[i].center + cells[j].center);
            if (!env.checkOccupancy(mid.x(), mid.y())) {
                double w = (cells[i].center - cells[j].center).norm();
                roadmap.addEdge(cells[i].center_id, cells[j].center_id, w);
                roadmap.addEdge(cells[j].center_id, cells[i].center_id, w);
            }
        }
    }

    // Connect special nodes
    auto connectSpecialToCenters = [&](int special_id, const Eigen::Vector2d& spos) {
        std::vector<std::pair<double,int>> cand;
        cand.reserve(cells.size());
        for (const auto& c : cells) cand.emplace_back((spos - c.center).norm(), c.center_id);
        std::sort(cand.begin(), cand.end());
        for (const auto& [d, cid] : cand) {
            const Eigen::Vector2d center_pos = roadmap.getNodes().at(cid).position;
            if (collisionFreeSegment(env, spos, center_pos)) {
                roadmap.addEdge(special_id, cid, d);
                roadmap.addEdge(cid, special_id, d);
                return true;
            }
        }
        if (!cand.empty()) {
            int fallback_id = cand.front().second;
            double fallback_dist = cand.front().first;
            ROS_WARN("Special node %d not collision-connected to any center. Falling back to nearest center %d (may be invalid).", special_id, fallback_id);
            roadmap.addEdge(special_id, fallback_id, fallback_dist);
            roadmap.addEdge(fallback_id, special_id, fallback_dist);
            return true;
        }
        ROS_ERROR("Special node %d: no centers to connect to.", special_id);
        return false;
    };

    if (!connectSpecialToCenters(0, start)) {
        ROS_ERROR("Failed to connect START to any center. Aborting.");
        return roadmap;
    }
    if (!connectSpecialToCenters(1, goal)) {
        ROS_ERROR("Failed to connect GOAL to any center. Aborting.");
        return roadmap;
    }

    // Connect victims to K nearest nodes
    for (size_t vi = 0; vi < victims.size(); ++vi) {
        int victim_id = static_cast<int>(2 + vi);
        const Eigen::Vector2d victim_pos = roadmap.getNodes().at(victim_id).position;
        std::vector<std::pair<double,int>> cand;
        cand.reserve(cells.size());
        for (const auto& c : cells) cand.emplace_back((victim_pos - c.center).norm(), c.center_id);
        std::sort(cand.begin(), cand.end());
        int connected = 0;
        for (size_t k = 0; k < cand.size() && connected < k_victim_neighbors_; ++k) {
            int cid = cand[k].second;
            const Eigen::Vector2d center_pos = roadmap.getNodes().at(cid).position;
            double dist = cand[k].first;
            if (collisionFreeSegment(env, victim_pos, center_pos)) {
                roadmap.addEdge(victim_id, cid, dist);
                roadmap.addEdge(cid, victim_id, dist);
                ++connected;
            }
        }
        if (connected == 0) {
            if (!cand.empty()) {
                int cid = cand.front().second;
                double dist = cand.front().first;
                ROS_WARN("Victim node %d had zero collision-free center connections; falling back to nearest center %d.", victim_id, cid);
                roadmap.addEdge(victim_id, cid, dist);
                roadmap.addEdge(cid, victim_id, dist);
            } else {
                ROS_ERROR("Victim node %d: no centers exist to connect to.", victim_id);
            }
        }
    }

    // Visualization
    visualization_msgs::MarkerArray marray;
    int mid = 0;
    for (const auto& c : cells) {
        const Quad& q = c.q;
        visualization_msgs::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.ns = "quadtree_cells";
        mk.id = mid++;
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = 0.5*(q.xl + q.xr);
        mk.pose.position.y = 0.5*(q.yl + q.yr);
        mk.pose.position.z = 0.02;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = (q.xr - q.xl) * 0.98;
        mk.scale.y = (q.yr - q.yl) * 0.98;
        mk.scale.z = 0.04;
        mk.color.r = 0.2; mk.color.g = 0.5; mk.color.b = 1.0; mk.color.a = 0.25;
        mk.lifetime = ros::Duration(0);
        marray.markers.push_back(mk);

        visualization_msgs::Marker cmk;
        cmk.header = mk.header;
        cmk.ns = "quadtree_centers";
        cmk.id = mid++;
        cmk.type = visualization_msgs::Marker::SPHERE;
        cmk.action = visualization_msgs::Marker::ADD;
        cmk.pose.position.x = c.center.x();
        cmk.pose.position.y = c.center.y();
        cmk.pose.position.z = 0.06;
        cmk.scale.x = 0.08; cmk.scale.y = 0.08; cmk.scale.z = 0.08;
        cmk.color.r = 1.0; cmk.color.g = 0.2; cmk.color.b = 0.2; cmk.color.a = 1.0;
        cmk.lifetime = ros::Duration(0);
        marray.markers.push_back(cmk);
    }
    slab_pub_.publish(marray);

    ROS_INFO("ACD Quadtree built: centers=%zu, roadmap nodes=%zu", cells.size(), roadmap.getNodes().size());
    return roadmap;
}

PLUGINLIB_EXPORT_CLASS(PlannerACD, PlannerBase)
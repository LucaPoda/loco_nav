// planner_ecd.cpp  -- Quadtree-based ECD with robust connectivity
#include "planners/planner_ecd.hpp"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <limits>
#include <algorithm>
#include <queue>

PlannerECD::PlannerECD() {}
PlannerECD::~PlannerECD() {}

void PlannerECD::initialize(const std::string& robot_name) {
    PlannerBase::initialize(robot_name);

    ros::NodeHandle nh;
    slab_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/ecd/quadtree", 1, true);

    ros::NodeHandle pnh("~");
    pnh.param("quadtree_max_depth", quadtree_max_depth_, 6);
    pnh.param("quadtree_min_size", quadtree_min_size_, 0.25);
    pnh.param("quadtree_samples_per_side", quadtree_samples_per_side_, 3);
    pnh.param("k_victim_neighbors", k_victim_neighbors_, 15);

    ROS_INFO("Initializing ECD (Quadtree) Planner plugin...");
    ROS_INFO("  quadtree_max_depth: %d", quadtree_max_depth_);
    ROS_INFO("  quadtree_min_size:  %.3f", quadtree_min_size_);
    ROS_INFO("  quadtree_samples:   %d", quadtree_samples_per_side_);
    ROS_INFO("  k_victim_neighbors: %d", k_victim_neighbors_);
}

// simple segment collision check by sampling along the straight line
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

struct Quad { double xl, xr, yl, yr; int depth; };

// return true if quad extends outside axis-aligned map bbox
static bool quadOutsideBounds(const Quad& q, double min_x, double max_x, double min_y, double max_y) {
    const double EPS = 1e-9;
    return (q.xl < min_x - EPS) || (q.xr > max_x + EPS) || (q.yl < min_y - EPS) || (q.yr > max_y + EPS);
}

enum class QuadStatus { FREE, OCCUPIED, MIXED };

// simple sampling to classify quad occupancy
static QuadStatus sampleQuad(const EnvironmentHandler& env, const Quad& q, int samples_per_side) {
    int occ = 0, freec = 0;
    for (int i = 0; i < samples_per_side; ++i) {
        double sx = q.xl + (i + 0.5)*(q.xr - q.xl)/samples_per_side;
        for (int j = 0; j < samples_per_side; ++j) {
            double sy = q.yl + (j + 0.5)*(q.yr - q.yl)/samples_per_side;
            if (env.checkOccupancy(sx, sy)) ++occ; else ++freec;
        }
    }
    if (occ == 0) return QuadStatus::FREE;
    if (freec == 0) return QuadStatus::OCCUPIED;
    return QuadStatus::MIXED;
}

// two quads are adjacent if they share a non-zero length edge
static bool quadsAreAdjacent(const Quad& a, const Quad& b) {
    const double EPS = 1e-9;
    // horizontal adjacency: a.xr == b.xl or b.xr == a.xl
    if (std::abs(a.xr - b.xl) < EPS || std::abs(b.xr - a.xl) < EPS) {
        double lo = std::max(a.yl, b.yl);
        double hi = std::min(a.yr, b.yr);
        return hi - lo > EPS;
    }
    // vertical adjacency: a.yr == b.yl or b.yr == a.yl
    if (std::abs(a.yr - b.yl) < EPS || std::abs(b.yr - a.yl) < EPS) {
        double lo = std::max(a.xl, b.xl);
        double hi = std::min(a.xr, b.xr);
        return hi - lo > EPS;
    }
    return false;
}

Roadmap PlannerECD::buildRoadmap() {
    Roadmap roadmap;
    const auto& env = getEnvironment();
    const auto& params = getParams();

    ROS_INFO("PlannerECD (Quadtree): building roadmap...");

    // 1) Add special nodes: start (0), goal (1)
    Eigen::Vector2d start = env.getStartPose().head<2>();
    Eigen::Vector2d goal  = env.getGoalPose().head<2>();
    roadmap.addNode(0, start, 0.0);
    roadmap.addNode(1, goal,  0.0);

    // 2) Add victims with ids 2..(2+V-1)
    const auto& victims = env.getVictims();
    int next_id = 2;
    for (size_t i = 0; i < victims.size(); ++i) {
        int vid = next_id++;
        roadmap.addNode(vid, victims[i].position, victims[i].reward);
    }

    // 3) Build root square covering map bbox
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

    // 4) Quadtree subdivision (stack)
    std::vector<Quad> leaves;
    std::vector<Quad> to_process;
    to_process.push_back(root);

    while (!to_process.empty()) {
        Quad q = to_process.back();
        to_process.pop_back();

        double q_w = q.xr - q.xl;
        double q_h = q.yr - q.yl;
        double size_min = std::min(q_w, q_h);

        bool outside = quadOutsideBounds(q, min_x, max_x, min_y, max_y);
        QuadStatus status = sampleQuad(env, q, quadtree_samples_per_side_);

        bool should_split = false;
        if (q.depth < quadtree_max_depth_ && size_min > quadtree_min_size_) {
            if (outside) should_split = true;
            else if (status == QuadStatus::MIXED) should_split = true;
            else if (status == QuadStatus::OCCUPIED) should_split = true;
        }
        if (q.depth <= 2) should_split = true;

        if (!should_split) {
            if (status == QuadStatus::FREE && !outside) leaves.push_back(q);
            continue;
        }

        double xm = 0.5*(q.xl + q.xr);
        double ym = 0.5*(q.yl + q.yr);
        Quad q1{q.xl, xm, q.yl, ym, q.depth + 1}; // bl
        Quad q2{xm, q.xr, q.yl, ym, q.depth + 1}; // br
        Quad q3{q.xl, xm, ym, q.yr, q.depth + 1}; // tl
        Quad q4{xm, q.xr, ym, q.yr, q.depth + 1}; // tr
        to_process.push_back(q1);
        to_process.push_back(q2);
        to_process.push_back(q3);
        to_process.push_back(q4);
    }

    ROS_INFO("Quadtree produced %zu leaf cells", leaves.size());
    if (leaves.empty()) {
        ROS_ERROR("Quadtree produced zero free leaves. Aborting roadmap construction.");
        visualization_msgs::MarkerArray empty_ma;
        slab_pub_.publish(empty_ma);
        return roadmap; // empty roadmap - upstream will notice
    }

    // 5) Create a center node per leaf (record CellInfo)
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

    // 6) Connect adjacent cell centers (collision-checked)
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

    // 7) Ensure start and goal are connected to at least one center
    auto connectSpecialToCenters = [&](int special_id, const Eigen::Vector2d& spos) {
        // build distance list to centers
        std::vector<std::pair<double,int>> cand;
        cand.reserve(cells.size());
        for (const auto& c : cells) cand.emplace_back((spos - c.center).norm(), c.center_id);
        std::sort(cand.begin(), cand.end());

        // try collision-checked nearest first; if none, fallback to nearest (warn)
        for (const auto& [d, cid] : cand) {
            // get center position from roadmap to be safe
            const Eigen::Vector2d center_pos = roadmap.getNodes().at(cid).position;
            if (collisionFreeSegment(env, spos, center_pos)) {
                roadmap.addEdge(special_id, cid, d);
                roadmap.addEdge(cid, special_id, d);
                return true;
            }
        }
        // fallback: connect to nearest center ignoring collisions (avoid crash). Warn loudly.
        if (!cand.empty()) {
            int fallback_id = cand.front().second;
            double fallback_dist = cand.front().first;
            ROS_WARN("Special node %d not collision-connected to any center. Falling back to nearest center %d (may be invalid).", special_id, fallback_id);
            roadmap.addEdge(special_id, fallback_id, fallback_dist);
            roadmap.addEdge(fallback_id, special_id, fallback_dist);
            return true;
        }
        // no centers available at all
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

    // 8) Connect victims to up to k_victim_neighbors centers (collision-checked). If none, fallback to nearest center (warn).
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
            // fallback to nearest center regardless of collision (avoid crash); warn
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

    // 9) Publish visualization markers: cells + centers
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

    ROS_INFO("ECD Quadtree built: centers=%zu, roadmap nodes=%zu", cells.size(), roadmap.getNodes().size());
    return roadmap;
}

PLUGINLIB_EXPORT_CLASS(PlannerECD, PlannerBase)
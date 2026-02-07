#ifndef PLANNER_ACD_H
#define PLANNER_ACD_H

#include "planners/planner_base.hpp"
#include <ros/ros.h>

class PlannerACD : public PlannerBase {
public:
    PlannerACD();

    virtual ~PlannerACD();

    void initialize(const std::string& robot_name) override;

protected:
    Roadmap buildRoadmap() override;

private:
    ros::Publisher slab_pub_;
    int quadtree_max_depth_;
    double quadtree_min_size_;
    int quadtree_samples_per_side_;
    int k_victim_neighbors_;
    double min_connection_distance_;
};

#endif // PLANNER_ACD_H
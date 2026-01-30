#ifndef PLANNER_PRM_H
#define PLANNER_PRM_H

#include "planners/planner_base.hpp"
#include <ros/ros.h>

class PlannerPRM : public PlannerBase {
public:
    // 1. Default Constructor (Required by pluginlib)
    PlannerPRM();

    // 2. Destructor
    virtual ~PlannerPRM();

    // 3. Initialization Override
    // We override this to load PRM-specific parameters
    void initialize(const std::string& robot_name) override;

protected:
    // 4. The Core Logic Override
    Roadmap buildRoadmap() override;

private:
    // Add your algorithm-specific parameters here
    
};
#endif
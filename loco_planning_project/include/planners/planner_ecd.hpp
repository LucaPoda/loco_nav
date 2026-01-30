#ifndef PLANNER_ECD_H
#define PLANNER_ECD_H

#include "planners/planner_base.hpp"
#include <ros/ros.h>

class PlannerECD : public PlannerBase {
public:
    // 1. Default Constructor (Required by pluginlib)
    PlannerECD();

    // 2. Destructor
    virtual ~PlannerECD();

    // 3. Initialization Override
    // We override this to load ecd-specific parameters
    void initialize(const std::string& robot_name) override;

protected:
    // 4. The Core Logic Override
    Roadmap buildRoadmap() override;

private:
    // Add your algorithm-specific parameters here

};

#endif // PLANNER_ECD_H
#ifndef DUBINS_OMPL_H
#define DUBINS_OMPL_H

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <Eigen/Dense>
#include <vector>

class DubinsGenerator {
public:
    /**
     * @brief Generates a discrete Dubins path between two points using OMPL
     * @param start (x, y, theta)
     * @param goal (x, y, theta)
     * @param Kmax Maximum curvature
     * @param step_size Discretization step (in meters)
     * @return Vector of points
     */
    static std::vector<Eigen::Vector3d> getPath(
        const Eigen::Vector3d& start, 
        const Eigen::Vector3d& goal, 
        double Kmax, 
        double step_size = 0.05) 
    {
        std::vector<Eigen::Vector3d> path_points;

        // 1. OMPL uses Turning Radius (rho), not Curvature (K)
        double rho = 1.0 / Kmax;

        // 2. Define the State Space
        // DubinsStateSpace is a specific type of SE2 (2D Position + Orientation)
        auto space = std::make_shared<ompl::base::DubinsStateSpace>(rho);

        // 3. Define Start and Goal States
        ompl::base::ScopedState<> start_ompl(space);
        ompl::base::ScopedState<> goal_ompl(space);

        // OMPL uses internal indices: 0->X, 1->Y, 2->Yaw
        start_ompl[0] = start.x(); 
        start_ompl[1] = start.y(); 
        start_ompl[2] = start.z(); // z holds theta in your vector

        goal_ompl[0] = goal.x(); 
        goal_ompl[1] = goal.y(); 
        goal_ompl[2] = goal.z();

        // 4. Calculate Distance (Length of the Dubins curve)
        double length = space->distance(start_ompl.get(), goal_ompl.get());

        // 5. Interpolate (Discretize the path)
        // We calculate how many steps we need
        int steps = std::max(2, (int)(length / step_size));

        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps; // t goes from 0.0 to 1.0
            
            ompl::base::State* state = space->allocState();
            space->interpolate(start_ompl.get(), goal_ompl.get(), t, state);

            // Extract values back to Eigen
            // We need to cast the generic State* to an SE2StateSpace::StateType*
            const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
            
            double x = se2state->getX();
            double y = se2state->getY();
            double yaw = se2state->getYaw();

            path_points.emplace_back(x, y, yaw);

            space->freeState(state);
        }

        return path_points;
    }
    
    /**
     * @brief Gets just the length (useful for the Distance Matrix/Dijkstra step)
     */
    static double getLength(
        const Eigen::Vector3d& start, 
        const Eigen::Vector3d& goal, 
        double Kmax) 
    {
        double rho = 1.0 / Kmax;
        auto space = std::make_shared<ompl::base::DubinsStateSpace>(rho);
        
        ompl::base::ScopedState<> s1(space), s2(space);
        s1[0]=start.x(); s1[1]=start.y(); s1[2]=start.z();
        s2[0]=goal.x();  s2[1]=goal.y();  s2[2]=goal.z();

        return space->distance(s1.get(), s2.get());
    }
};

#endif
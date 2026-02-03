// Orienteering.cpp
#include "orienteering/Orienteering.hpp"
#include "ortools/linear_solver/linear_solver.h"
#include <map>
#include <set>

using namespace operations_research;

std::vector<int> OrienteeringPlanner::plan(const Roadmap& roadmap, int start_id, int end_id, double T_max) {
    using namespace operations_research;
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));
    if (!solver) return {};

    const auto& nodes = roadmap.getNodes();
    Eigen::Vector2d start_pos = nodes.at(start_id).position;
    Eigen::Vector2d end_pos = nodes.at(end_id).position;

    // --- 1. PRE-SOLVE PRUNING (Ellipse Heuristic) ---
    // Remove nodes that are mathematically impossible to visit
    std::set<int> valid_ids;
    for (auto const& [id, node] : nodes) {
        double min_dist = (node.position - start_pos).norm() + (end_pos - node.position).norm();
        if (min_dist <= T_max) {
            valid_ids.insert(id);
        }
    }
    int N = valid_ids.size();

    // --- 2. SOLVER CONFIGURATION ---
    // Set Relative Gap to 5% (Stops the solver once it's "close enough")
    MPSolverParameters params;
    params.SetDoubleParam(MPSolverParameters::RELATIVE_MIP_GAP, 0.05);
    
    // Safety timeout of 1 second
    solver->set_time_limit(5000);

    // --- 3. VARIABLES ---
    std::map<int, MPVariable*> x; 
    std::map<int, std::map<int, MPVariable*>> e; 
    std::map<int, MPVariable*> u; 

    for (int id : valid_ids) {
        x[id] = solver->MakeIntVar(0.0, 1.0, "x_" + std::to_string(id));
        
        // TIGHTENED MTZ BOUNDS: Start is 1, others are [2, N]
        if (id == start_id) u[id] = solver->MakeNumVar(1.0, 1.0, "u_" + std::to_string(id));
        else u[id] = solver->MakeNumVar(2.0, (double)N, "u_" + std::to_string(id));

        for (const auto& edge : roadmap.getNeighbors(id)) {
            if (valid_ids.count(edge.to)) {
                e[id][edge.to] = solver->MakeIntVar(0.0, 1.0, "e_" + std::to_string(id) + "_" + std::to_string(edge.to));
            }
        }
    }

    // --- 4. CONSTRAINTS ---
    solver->MakeRowConstraint(1.0, 1.0)->SetCoefficient(x[start_id], 1.0);
    solver->MakeRowConstraint(1.0, 1.0)->SetCoefficient(x[end_id], 1.0);

    for (int i : valid_ids) {
        MPConstraint* flow = solver->MakeRowConstraint(0.0, 0.0);
        for (auto const& [to_id, var] : e[i]) flow->SetCoefficient(var, 1.0); // Out
        for (int prev_id : valid_ids) {
            if (e[prev_id].count(i)) flow->SetCoefficient(e[prev_id][i], -1.0); // In
        }

        if (i == start_id) flow->SetBounds(1.0, 1.0);
        else if (i == end_id) flow->SetBounds(-1.0, -1.0);
        else {
            // Visit Constraint: If visited (x[i]=1), flow must balance. 
            // If not visited (x[i]=0), no flow allowed.
            MPConstraint* visit = solver->MakeRowConstraint(0.0, 0.0);
            visit->SetCoefficient(x[i], -1.0);
            for (auto const& [to_id, var] : e[i]) visit->SetCoefficient(var, 1.0);
        }

        // Subtour Elimination
        for (auto const& [j, edge_var] : e[i]) {
            if (j == start_id) continue;
            MPConstraint* mtz = solver->MakeRowConstraint(-MPSolver::infinity(), (double)N - 1);
            mtz->SetCoefficient(u[i], 1.0);
            mtz->SetCoefficient(u[j], -1.0);
            mtz->SetCoefficient(edge_var, (double)N);
        }
    }

    // Budget
    MPConstraint* budget = solver->MakeRowConstraint(0.0, T_max);
    for (int i : valid_ids) {
        for (const auto& edge : roadmap.getNeighbors(i)) {
            if (e[i].count(edge.to)) budget->SetCoefficient(e[i][edge.to], edge.weight);
        }
    }

    // --- 5. OBJECTIVE ---
    MPObjective* const obj = solver->MutableObjective();
    for (int id : valid_ids) obj->SetCoefficient(x[id], nodes.at(id).score);
    obj->SetMaximization();

    solver->Solve(params); // Pass the parameters here

    // --- 6. EXTRACTION ---
    std::vector<int> path;
    int curr = start_id;
    path.push_back(curr);
    while (curr != end_id) {
        bool found = false;
        for (auto const& [next_id, var] : e[curr]) {
            if (var->solution_value() > 0.5) {
                curr = next_id;
                path.push_back(curr);
                found = true;
                break;
            }
        }
        if (!found) break;
    }
    return path;
}
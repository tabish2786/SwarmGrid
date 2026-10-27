#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M8: lifelong warehouse simulation with Rolling-Horizon Collision Resolution (RHCR,
// Li et al., 2021).
//
// Orders appear at pickup cells (free cells next to a shelf). A free robot is assigned an order,
// drives to its pickup, then to a drop-off station on the outer ring of the map; arriving there
// delivers the order and frees the robot. The number of waiting (unassigned) orders is kept at
// `backlog`, so robots are never short of work: throughput measures the fleet, not the demand.
//
// Every `replanEvery` (h) steps, the whole fleet is replanned from where it stands, resolving
// collisions only for the first `window` (w) timesteps (w >= h); then h steps are executed.
// That is what lets ECBS/CBS keep up with many robots: conflicts far in the future are not
// worth resolving, they will be replanned anyway.
//
// Things that go wrong, and what the simulator does:
//   - planner fails (node limit): fall back to windowed prioritized planning; if that fails as
//     well, every robot waits one step (always collision-free) and planning is retried.
//   - deadlock / livelock: a robot that has not got closer to its goal for `stuckSteps` steps is
//     stuck. Each stuck robot is sent to a nearby free "escape" cell, then resumes its task.
//   - breakdowns: each step, each working robot breaks down with probability breakdownRate and
//     stands still for repairSteps steps. To the planner it is a shelf; an order it had not picked
//     up yet goes back to the waiting list. The fleet replans at once around it.
// Every executed step is checked for collisions (the count must stay 0).

enum class LifelongSolver { Prioritized, Cbs, Ecbs };
enum class AssignPolicy { Greedy, Hungarian };

struct LifelongOptions {
    int robots = 20;
    int steps = 500;
    int window = 10;         // w; -1 = resolve whole paths
    int replanEvery = 5;     // h, 1 <= h, and h <= w when w >= 0
    LifelongSolver solver = LifelongSolver::Ecbs;
    double ecbsWeight = 1.2;
    AssignPolicy assignment = AssignPolicy::Hungarian;
    int backlog = 0;         // waiting orders kept open; 0 = same as robots
    double breakdownRate = 0.0;
    int repairSteps = 20;
    int stuckSteps = 40;
    int plannerMaxNodes = 2000;
    double plannerTimeLimit = 0.0;  // seconds per replan, 0 = node limit only (deterministic)
    std::uint32_t seed = 1;
};

struct LifelongStats {
    int steps = 0;
    int delivered = 0;
    double throughput = 0.0;          // delivered per timestep
    int replans = 0;
    int plannerFailures = 0;          // main solver gave up (fallback used)
    int allWait = 0;                  // the fallback failed too: everyone waited one step
    int deadlocks = 0;                // replans that found newly stuck robots
    int escapes = 0;                  // robots sent to an escape cell
    int breakdowns = 0;
    int collisions = 0;               // vertex or swap conflicts in the executed motion (want 0)
    long long assignedDistance = 0;   // sum of robot -> pickup distances at assignment time
    double meanPlanSeconds = 0.0;
    double maxPlanSeconds = 0.0;
    // positions[t][i]: robot i's cell at time t (t = 0 .. steps). Only if recordPositions.
    std::vector<std::vector<CellId>> positions;
};

// Deterministic for a fixed seed when plannerTimeLimit == 0.
// Throws std::invalid_argument for robots < 1, steps < 1, bad window/replanEvery, or a grid
// without enough free cells, pickup cells or stations.
LifelongStats runLifelong(const Grid& grid, const LifelongOptions& options,
                          bool recordPositions = false);

const char* solverName(LifelongSolver s);
const char* assignName(AssignPolicy a);

}  // namespace swarmgrid

#pragma once

#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M4: Conflict-Based Search (Sharon et al., 2015). Optimal for sum-of-costs.
//
// High level: a Constraint Tree (CT), searched best-first on sum-of-costs. Each CT node holds a
// set of constraints and one path per agent that respects them (from space-time A*, M2).
// Pop the cheapest node; if its paths are conflict-free it is an optimal solution. Otherwise take
// its first conflict and create two children, each forbidding one of the two agents from doing
// its part of the conflict, and replan only that agent:
//   Vertex / Target (a1, a2, cell, t): child 1: a1 not on cell at t; child 2: a2 not on cell at t.
//     (For a target conflict, "a1 not on its goal at t" forces a1 to finish later, through the
//      goal condition of space-time A*.)
//   Edge (a1: cell->to, a2: to->cell, t): child 1 forbids a1's move, child 2 forbids a2's move.
// OPEN order: smaller sum-of-costs, then fewer conflicts, then older node. Deterministic.
struct CbsOptions {
    int maxNodes = 100000;          // CT nodes to expand before giving up
    double timeLimitSeconds = 0.0;  // wall-clock limit; 0 = none (results then depend only on input)
};

struct CbsResult {
    bool solved = false;
    bool limitReached = false;   // stopped by maxNodes or the time limit (not proven unsolvable)
    std::vector<Path> paths;     // paths[i] for agent i; empty unless solved
    long long sumOfCosts = 0;
    int nodesExpanded = 0;       // CT nodes popped from OPEN
    int nodesGenerated = 0;      // CT nodes created (root + children with a valid replan)
    long long lowLevelExpanded = 0;
    double seconds = 0.0;        // measured wall-clock time
};

// Throws std::invalid_argument for bad agents (see requireValidAgents) or maxNodes < 1.
CbsResult solveCbs(const Grid& grid, const std::vector<CellId>& starts,
                   const std::vector<CellId>& goals, const CbsOptions& options = {});

}  // namespace swarmgrid

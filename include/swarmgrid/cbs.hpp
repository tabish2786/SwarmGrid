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
//
// M5 (Boyarski et al., 2015, "ICBS"), both on by default and both keep CBS optimal:
//   prioritizeConflicts: classify every conflict of the node with MDDs (mdd.hpp). A conflict is
//     cardinal if BOTH children must get more expensive (each agent's optimal paths all use the
//     conflicting cell/move), semi-cardinal if one must, non-cardinal otherwise. Branch on the
//     first cardinal, else the first semi-cardinal, else the first conflict. Splitting on a
//     cardinal conflict raises the lower bound right away instead of growing a wide tree of
//     equal-cost nodes.
//   bypass: if a child has the parent's cost and fewer conflicts, take its new path into the
//     parent and put the parent back in OPEN instead of adding two children.
// With both off this is the plain CBS of M4 (the tests pin its node counts).
struct CbsOptions {
    int maxNodes = 100000;          // CT nodes to expand before giving up
    double timeLimitSeconds = 0.0;  // wall-clock limit; 0 = none (results then depend only on input)
    bool prioritizeConflicts = true;
    bool bypass = true;
    // M8 rolling horizon: if >= 0, only conflicts at t <= window are resolved (the plan is
    // collision-free up to then and each robot simply drives its shortest way after it).
    // Optimality then refers to that relaxed problem. -1 = whole paths (normal MAPF).
    int window = -1;
};

struct CbsResult {
    bool solved = false;
    bool limitReached = false;   // stopped by maxNodes or the time limit (not proven unsolvable)
    std::vector<Path> paths;     // paths[i] for agent i; empty unless solved
    long long sumOfCosts = 0;
    int nodesExpanded = 0;       // CT nodes popped from OPEN
    int nodesGenerated = 0;      // CT nodes created (root + children with a valid replan)
    long long lowLevelExpanded = 0;
    int bypasses = 0;            // M5: expansions that took a child's path instead of branching
    int cardinalSplits = 0;      // M5: conflicts picked per expansion: cardinal / semi / other
                                 // (counted before a possible bypass)
    int semiCardinalSplits = 0;
    int nonCardinalSplits = 0;
    int mddsBuilt = 0;
    double seconds = 0.0;        // measured wall-clock time
};

// Throws std::invalid_argument for bad agents (see requireValidAgents) or maxNodes < 1.
CbsResult solveCbs(const Grid& grid, const std::vector<CellId>& starts,
                   const std::vector<CellId>& goals, const CbsOptions& options = {});

}  // namespace swarmgrid

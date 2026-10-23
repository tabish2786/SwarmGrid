#pragma once

#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

enum class MultiAgentMode {
    // Every agent gets its own optimal A* path and ignores the others. Paths may collide.
    Independent,
    // Agents are planned one at a time with space-time A*. Each agent treats the agents planned
    // before it as moving obstacles:
    //   - Vertex constraints on every (cell, t) they occupy,
    //   - Edge constraints on the reverse of every move they make (so no swaps),
    //   - a BlockedFrom constraint on their goal from the time they park there.
    // Collision-free when it succeeds. NOT optimal, and NOT complete: a bad priority order can
    // fail on a solvable instance. On failure the failed agent moves to the front of the order
    // and planning restarts, up to maxAttempts orders. (CBS, milestone M4, is optimal.)
    Prioritized,
};

struct MultiAgentResult {
    bool solved = false;
    std::vector<Path> paths;    // paths[i] for agent i; empty if !solved
    int failedAgent = -1;       // agent that had no path in the last order tried (if !solved)
    std::vector<int> order;     // priority order of the last attempt (Prioritized only)
    int attempts = 0;           // priority orders tried (Prioritized), 1 for Independent
    long long expanded = 0;     // total low-level search expansions over all attempts
};

// Plans starts[i] -> goals[i] for every agent i. Deterministic.
// Throws std::invalid_argument if sizes differ, there are no agents, maxAttempts < 1, or any
// start/goal is not a free cell, or two agents share a start or a goal.
MultiAgentResult planMultiAgent(const Grid& grid, const std::vector<CellId>& starts,
                                const std::vector<CellId>& goals, MultiAgentMode mode,
                                int maxAttempts = 10);

}  // namespace swarmgrid

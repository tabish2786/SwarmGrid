#pragma once

#include <vector>

#include "swarmgrid/constraints.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M5: Multi-valued Decision Diagram of one agent (Sharon et al.). levels[t] holds every cell the
// agent can occupy at time t on SOME path that
//   - starts at `start`, ends at `goal` at exactly time `cost`,
//   - respects `constraints` (including the goal condition: cost > latest vertex constraint on
//     the goal, and no BlockedFrom on the goal).
// If levels[t] has a single cell, every optimal path goes through it at time t: forbidding it
// must make the agent's path longer. That is what makes a conflict "cardinal".
struct Mdd {
    std::vector<std::vector<CellId>> levels;  // levels[0..cost], each sorted; empty if no such path
};

// h must be bfsDistances(grid, goal) (used to prune cells that can't make it in time).
// Cost: O(sum of level sizes * 5). Throws std::invalid_argument for bad start/goal/h or cost < 0.
Mdd buildMdd(const Grid& grid, CellId start, CellId goal, int cost, const std::vector<int>& h,
             const ConstraintTable& constraints);

}  // namespace swarmgrid

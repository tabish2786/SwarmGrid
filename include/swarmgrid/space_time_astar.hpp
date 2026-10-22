#pragma once

#include <optional>
#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/constraints.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M2: optimal single-agent path in space-time that respects a ConstraintTable.
//
// State = (cell, t). Actions: 4 moves + wait, each costs 1, so g == t and the path cost is
// the arrival time T = path.size() - 1. path[t] == path[t+1] means "wait".
//
// Goal condition: at the goal at time T with T > constraints.latestVertexConstraintAt(goal).
// The agent then stays on its goal forever, so no later constraint there may exist.
//
// h: admissible static heuristic, normally bfsDistances(grid, goal); kUnreachable prunes.
// OPEN order: smaller f first, then larger t, then smaller cell id. Deterministic.
//
// horizon: no state with t > horizon is generated. horizon < 0 means automatic:
// constraints.maxTime() + grid.numCells(), which never cuts off a solution (see the .cpp).
// Returns std::nullopt if no valid path ends by the horizon.
// Throws std::invalid_argument for bad start/goal or h.size() != numCells() (like astar).
// Resets and fills *stats if non-null. Each state is generated at most once (g is fixed by
// t), so expanded counts distinct states.
std::optional<Path> spaceTimeAstar(const Grid& grid, CellId start, CellId goal,
                                   const std::vector<int>& h, const ConstraintTable& constraints,
                                   int horizon = -1, SearchStats* stats = nullptr);

}  // namespace swarmgrid

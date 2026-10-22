#pragma once

#include <optional>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// path[t] = cell occupied at timestep t. path.front() = start, path.back() = goal.
// Moves = path.size() - 1.
using Path = std::vector<CellId>;

struct SearchStats {
    // Nodes popped from OPEN and expanded, including the goal pop. Stale duplicates are skipped
    // and not counted. With an inconsistent heuristic a cell can be re-expanded; each time
    // counts, so expanded can exceed numCells().
    int expanded = 0;
    int generated = 0;  // nodes pushed onto OPEN, including the start
};

// M1: optimal single-agent shortest path on the static grid (no time dimension, no waits).
//
// h[c] must never overestimate the true distance from c to goal (admissible);
// h[c] == kUnreachable means "cannot reach the goal from c" and prunes that cell.
// OPEN order: smaller f = g + h first; ties -> larger g (closer to the goal);
// ties -> smaller cell id. Fully deterministic.
//
// Returns std::nullopt if the goal is unreachable.
// Throws std::invalid_argument if start/goal are not free cells or h.size() != numCells().
// Resets and fills *stats if non-null.
std::optional<Path> astar(const Grid& grid, CellId start, CellId goal, const std::vector<int>& h,
                          SearchStats* stats = nullptr);

}  // namespace swarmgrid

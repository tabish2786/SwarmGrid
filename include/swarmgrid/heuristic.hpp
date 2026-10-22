#pragma once

#include <limits>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// Distance value for cells that can never reach the goal (obstacles, walled-off regions).
// The extra parentheses stop a Windows max() macro (from <windows.h>) from expanding here.
constexpr int kUnreachable = (std::numeric_limits<int>::max)();

// Exact shortest distance (number of moves) from every cell to `goal`, from ONE BFS that
// starts at the goal. Works because moves are undirected with unit cost, so
// dist(c -> goal) == dist(goal -> c).
// dist[c] == kUnreachable for obstacles and cells that cannot reach the goal.
// O(V) time and memory, V = grid.numCells(). Meant to be built once per goal and reused by
// every search towards that goal (CBS replans the same agent many times).
// Throws std::invalid_argument if goal is out of range or not free.
std::vector<int> bfsDistances(const Grid& grid, CellId goal);

// Manhattan distance |dx| + |dy| from every free cell to `goal` (kUnreachable on obstacles).
// Admissible but blind to shelves. Kept only as a baseline to measure how much BFS helps.
// Throws std::invalid_argument if goal is out of range or not free.
std::vector<int> manhattanDistances(const Grid& grid, CellId goal);

}  // namespace swarmgrid

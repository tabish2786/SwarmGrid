#pragma once

#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// Which robot takes which order. Indices refer to the robotCells / orderCells passed in.
struct Assignment {
    int robot = -1;
    int order = -1;
    int distance = 0;  // true grid distance (BFS, around shelves) from the robot to the order
};

// Greedy nearest-pair assignment: repeatedly pick the closest (robot, order) pair among the
// robots and orders still free, until no free robot can reach a free order. Ties: smaller
// distance, then smaller order index, then smaller robot index. Deterministic.
//
// NOT optimal for total distance: the Hungarian algorithm would be (see the test for a case
// where greedy loses). Greedy is what many real fleet managers start with: simple, fast, and
// nearly as good when orders are spread out.
//
// Cost: one BFS per order, O(m * V), plus O(min(k, m) * k * m) to pick the pairs, for k robots
// and m orders on a grid with V cells.
// Throws std::invalid_argument if any robot or order cell is not a free cell.
std::vector<Assignment> assignNearest(const Grid& grid, const std::vector<CellId>& robotCells,
                                      const std::vector<CellId>& orderCells);

// Optimal assignment (Hungarian algorithm, Kuhn 1955 / Munkres 1957, the O(n^3)
// shortest-augmenting-path form). Among all ways to give each robot at most one order and each
// order at most one robot, it first assigns as many reachable pairs as possible, then minimises
// the total distance of those pairs. Pairs that cannot reach each other are never returned.
// Output sorted by order index. Deterministic.
//
// Cost: the same m BFS as assignNearest, plus O(n^3) for n = max(k, m) (the cost matrix is
// padded to square with zero-cost dummy rows or columns).
// Throws std::invalid_argument if any robot or order cell is not a free cell.
std::vector<Assignment> assignOptimal(const Grid& grid, const std::vector<CellId>& robotCells,
                                      const std::vector<CellId>& orderCells);

}  // namespace swarmgrid

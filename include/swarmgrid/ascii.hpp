#pragma once

#include <string>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// Functions RETURN strings instead of printing => exact-match testable.

// '.' free, '@' obstacle. One line per row, every line ends with '\n'.
std::string renderGrid(const Grid& grid);

// Agent symbol for index i:
//   0-9  -> '0'..'9'
//   10-35 -> 'a'..'z'
//   36-61 -> 'A'..'Z'
//   >= 62 -> '*'
char agentSymbol(int agentIndex);

// Like renderGrid, but cell holding agent i shows agentSymbol(i).
// Two or more agents on the same cell -> '!' (you'll love this when debugging conflicts).
// Agent on an obstacle or out of bounds -> throws std::invalid_argument
// (that's a bug upstream, fail loudly).
std::string renderAgents(const Grid& grid, const std::vector<Point>& positions);

}  // namespace swarmgrid

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

// Like renderGrid, but cells on `path` show '+', the first cell 'S' and the last cell 'G'
// (a one-cell path shows 'G'). Throws std::invalid_argument if a path cell is out of range
// or an obstacle.
std::string renderPath(const Grid& grid, const std::vector<CellId>& path);

// "(x,y)@t" for every timestep, space separated, e.g. "(0,0)@0 (1,0)@1 (1,0)@2" (a repeated
// cell is a wait). Throws std::invalid_argument for a cell id outside the grid.
std::string formatTimedPath(const Grid& grid, const std::vector<CellId>& path);

}  // namespace swarmgrid

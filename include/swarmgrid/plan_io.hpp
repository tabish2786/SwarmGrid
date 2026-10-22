#pragma once

#include <ostream>
#include <string>
#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/constraints.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// Everything the web dashboard (docs/index.html) needs to draw one plan.
struct PlanExport {
    std::string id;      // short machine name, e.g. "warehouse-10"
    std::string title;   // shown in the dashboard
    std::string solver;  // how the paths were computed, shown verbatim
    std::string note;    // honest caveats, shown verbatim
    Grid grid;
    std::vector<Point> starts;
    std::vector<Point> goals;
    std::vector<Path> paths;              // paths[i][t] = cell of agent i at time t
    std::vector<int> expanded;            // per-agent search expansions, or empty
    std::vector<Constraint> constraints;  // optional, drawn on the map
};

// Writes one plan as JSON, format "swarmgrid-plan-v1":
//   { "format", "id", "title", "solver", "note",
//     "map": { "width", "height", "rows": ["..@..", ...] },          '.' free, '@' obstacle
//     "agents": [ { "id", "start": [x,y], "goal": [x,y], "cost", "expanded",
//                   "path": [[x,y,t], ...] } ],                       "expanded" only if known
//     "constraints": [ { "agent", "type": "vertex"|"edge", "cell": [x,y], "to": [x,y], "t" } ],
//     "stats": { "agents", "sum_of_costs", "makespan" } }
// cost = path.size() - 1 (arrival time); sum_of_costs and makespan follow from it.
// Output is deterministic (fixed field order, no floating point).
// Throws std::invalid_argument if the vectors disagree in size, a path is empty, or a path,
// start, goal or constraint cell lies outside the grid.
void writePlanJson(std::ostream& out, const PlanExport& plan);

// Escapes s for use inside a JSON string literal (the surrounding quotes are not added).
std::string jsonEscape(const std::string& s);

}  // namespace swarmgrid

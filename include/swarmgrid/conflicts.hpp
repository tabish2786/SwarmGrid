#pragma once

#include <optional>
#include <string>
#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M3: conflicts between timed paths, and an independent plan validator.
//
// Convention everywhere: path[t] is the agent's cell at time t, and after its path ends an agent
// stays on its last cell (its goal) forever.

enum class ConflictType {
    Vertex,  // a1 and a2 on `cell` at time t, both still moving
    Edge,    // a1 moves cell -> to while a2 moves to -> cell, arriving at time t (a swap)
    Target,  // a1 is already parked on its goal `cell`; a2 is there at time t
};

struct Conflict {
    ConflictType type = ConflictType::Vertex;
    int a1 = -1;
    int a2 = -1;
    CellId cell = kInvalidCell;
    CellId to = kInvalidCell;  // Edge only
    int t = 0;
};

// The earliest conflict: smallest t, then smallest (a1, a2) pair, a vertex/target conflict
// before an edge conflict at the same t. Deterministic. std::nullopt if the paths are
// conflict-free. O(T * n^2) for n agents and makespan T.
std::optional<Conflict> findFirstConflict(const std::vector<Path>& paths);

// Every conflict, in the same order findFirstConflict would report them.
std::vector<Conflict> findAllConflicts(const std::vector<Path>& paths);

struct ValidationReport {
    bool ok = false;                  // no errors and no conflicts
    std::vector<std::string> errors;  // malformed paths (wrong start/goal, jumps, obstacles, ...)
    std::vector<Conflict> conflicts;
    long long sumOfCosts = 0;         // sum of (path.size() - 1)
    int makespan = 0;                 // max of (path.size() - 1)
};

// Re-checks a finished plan without trusting any solver: right number of paths, each starts at
// its start and ends at its goal, every step is a 4-neighbour move or a wait onto a free cell,
// and no two agents conflict. Uses only Grid and findAllConflicts, never the search code.
ValidationReport validatePlan(const Grid& grid, const std::vector<CellId>& starts,
                              const std::vector<CellId>& goals, const std::vector<Path>& paths);

// Throws std::invalid_argument (message starts with `who`) unless starts and goals have the same
// non-zero size, every start and goal is a free cell, and no two agents share a start or a goal.
void requireValidAgents(const Grid& grid, const std::vector<CellId>& starts,
                        const std::vector<CellId>& goals, const std::string& who);

}  // namespace swarmgrid

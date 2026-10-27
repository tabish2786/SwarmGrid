#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/constraints.hpp"
#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// M6: Enhanced CBS (Barer et al., 2014). Bounded-suboptimal: the plan's sum of costs is at most
// `weight` times the optimum, and in return it scales to many more robots than CBS.
//
// Both levels use FOCAL search. Next to the usual OPEN list ordered by a lower bound, FOCAL holds
// every entry whose cost is within `weight` of the smallest lower bound in OPEN, and is ordered by
// "how many conflicts with the other robots". So the search may take a slightly longer path or
// CT node when that avoids collisions, but never one more than `weight` times worse than the
// best still possible. weight = 1 gives an optimal plan (it is CBS with a different tie-break).

// How often the OTHER robots' paths use each cell / move: the "conflict avoidance table" the
// low level counts against. Robots that have finished stay on their goal forever.
class ConflictAvoidanceTable {
public:
    // window >= 0: only the part of the path up to time `window` is recorded (rolling horizon).
    void addPath(const Path& path, int window = -1);

    // Conflicts caused by being on `to` at time t after being on `from` at t-1:
    // others on `to` at t (moving or parked there), plus others moving to -> from at t (a swap).
    int stepConflicts(CellId from, CellId to, int t) const;

    // Visits by other robots to `cell` at any time > t: what parking on `cell` from t would hit.
    int visitsAfter(CellId cell, int t) const;

private:
    static std::uint64_t vertexKey(CellId cell, int t);
    struct MoveKey {
        CellId from;
        CellId to;
        int t;
        bool operator==(const MoveKey& o) const { return from == o.from && to == o.to && t == o.t; }
    };
    struct MoveKeyHash {
        std::size_t operator()(const MoveKey& k) const noexcept;
    };
    // Lookups only, never iterated: results don't depend on the standard library's hash order.
    std::unordered_map<std::uint64_t, int> moving_;               // (cell, t) -> robots there
    std::unordered_map<MoveKey, int, MoveKeyHash> moves_;         // (from, to, t) -> robots
    std::unordered_map<CellId, std::vector<int>> parkedFrom_;     // cell -> times robots park
    std::unordered_map<CellId, std::vector<int>> visitTimes_;     // cell -> sorted times (moving)
};

struct FocalPath {
    Path path;
    int lowerBound = 0;  // no path for this agent under these constraints is shorter
    int conflicts = 0;   // with the table, including after it parks on its goal
};

// Space-time focal search, same model and goal condition as spaceTimeAstar (M2). Returns a path
// with length <= weight * lowerBound, preferring fewer conflicts with `cat`; std::nullopt if no
// valid path exists by the automatic horizon. With weight = 1 the length is optimal.
// Throws std::invalid_argument for bad start/goal, h.size() != numCells(), or weight < 1.
std::optional<FocalPath> focalSpaceTimeSearch(const Grid& grid, CellId start, CellId goal,
                                              const std::vector<int>& h,
                                              const ConstraintTable& constraints,
                                              const ConflictAvoidanceTable& cat, double weight,
                                              SearchStats* stats = nullptr);

struct EcbsOptions {
    double weight = 1.2;            // suboptimality bound, >= 1
    int maxNodes = 100000;          // CT nodes to expand before giving up
    double timeLimitSeconds = 0.0;  // 0 = none
    int window = -1;                // M8 rolling horizon, as in CbsOptions; -1 = whole paths
};

struct EcbsResult {
    bool solved = false;
    bool limitReached = false;
    std::vector<Path> paths;
    long long sumOfCosts = 0;
    long long lowerBound = 0;    // optimal sum of costs >= lowerBound, and sumOfCosts <= weight * it
    int nodesExpanded = 0;
    int nodesGenerated = 0;
    long long lowLevelExpanded = 0;
    double seconds = 0.0;
};

// Throws std::invalid_argument for bad agents, maxNodes < 1 or weight < 1.
EcbsResult solveEcbs(const Grid& grid, const std::vector<CellId>& starts,
                     const std::vector<CellId>& goals, const EcbsOptions& options = {});

}  // namespace swarmgrid

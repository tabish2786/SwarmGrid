#include "swarmgrid/mdd.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <unordered_set>

#include "swarmgrid/heuristic.hpp"

namespace swarmgrid {

Mdd buildMdd(const Grid& grid, CellId start, CellId goal, int cost, const std::vector<int>& h,
             const ConstraintTable& constraints) {
    const int n = grid.numCells();
    auto isFreeCell = [&](CellId c) { return c >= 0 && c < n && grid.isFree(c); };
    if (!isFreeCell(start) || !isFreeCell(goal) || cost < 0 || static_cast<int>(h.size()) != n) {
        throw std::invalid_argument("buildMdd: bad start, goal, cost or heuristic table");
    }
    Mdd mdd;
    const bool goalUsable = constraints.blockedFromAt(goal) < 0 &&
                            cost > constraints.latestVertexConstraintAt(goal) &&
                            !constraints.vertexBlocked(goal, cost);
    if (!goalUsable || constraints.vertexBlocked(start, 0) || h[start] > cost) return mdd;

    // Forward: cells reachable at time t that can still reach the goal by `cost` (static distance).
    std::vector<std::vector<CellId>> forward(static_cast<std::size_t>(cost) + 1);
    forward[0] = {start};
    std::vector<CellId> next;
    for (int t = 0; t < cost; ++t) {
        std::unordered_set<CellId> seen;
        for (CellId c : forward[static_cast<std::size_t>(t)]) {
            grid.neighbors(c, next);
            next.push_back(c);  // wait
            for (CellId m : next) {
                if (h[m] == kUnreachable || h[m] > cost - (t + 1)) continue;
                if (constraints.vertexBlocked(m, t + 1)) continue;
                if (m != c && constraints.edgeBlocked(c, m, t + 1)) continue;
                seen.insert(m);
            }
        }
        forward[static_cast<std::size_t>(t) + 1].assign(seen.begin(), seen.end());
    }

    // Backward: keep only cells with a valid move into a kept cell of the next level.
    std::vector<std::vector<CellId>> levels(static_cast<std::size_t>(cost) + 1);
    const auto& last = forward[static_cast<std::size_t>(cost)];
    if (std::find(last.begin(), last.end(), goal) == last.end()) return mdd;
    levels[static_cast<std::size_t>(cost)] = {goal};
    for (int t = cost - 1; t >= 0; --t) {
        const std::unordered_set<CellId> keep(levels[static_cast<std::size_t>(t) + 1].begin(),
                                              levels[static_cast<std::size_t>(t) + 1].end());
        for (CellId c : forward[static_cast<std::size_t>(t)]) {
            grid.neighbors(c, next);
            next.push_back(c);
            for (CellId m : next) {
                if (keep.count(m) && (m == c || !constraints.edgeBlocked(c, m, t + 1))) {
                    levels[static_cast<std::size_t>(t)].push_back(c);
                    break;
                }
            }
        }
        if (levels[static_cast<std::size_t>(t)].empty()) return mdd;
    }
    // Every kept cell is now on some valid start -> goal path; sort for deterministic output.
    for (auto& level : levels) std::sort(level.begin(), level.end());
    mdd.levels = std::move(levels);
    return mdd;
}

}  // namespace swarmgrid

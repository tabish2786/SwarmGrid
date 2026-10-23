#include "swarmgrid/space_time_astar.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "swarmgrid/heuristic.hpp"

namespace swarmgrid {

namespace {

struct Node {
    CellId cell;
    int t;
    int parent;  // index into the node pool, -1 for the start
};

struct OpenEntry {
    int f;
    int t;
    CellId cell;
    int index;  // into the node pool
};

// std::priority_queue pops the element that compares LARGEST: true means a pops after b.
struct PopsLater {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        if (a.f != b.f) return a.f > b.f;  // smaller f first
        if (a.t != b.t) return a.t < b.t;  // then larger t (deeper)
        return a.cell > b.cell;            // then smaller cell id
    }
};

std::uint64_t stateKey(CellId cell, int t) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t)) << 32) |
           static_cast<std::uint32_t>(cell);
}

Path reconstruct(const std::vector<Node>& pool, int index) {
    Path path;
    for (int i = index; i != -1; i = pool[static_cast<std::size_t>(i)].parent) {
        path.push_back(pool[static_cast<std::size_t>(i)].cell);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace

std::optional<Path> spaceTimeAstar(const Grid& grid, CellId start, CellId goal,
                                   const std::vector<int>& h, const ConstraintTable& constraints,
                                   int horizon, SearchStats* stats) {
    const int n = grid.numCells();
    auto isFreeCell = [&](CellId c) { return c >= 0 && c < n && grid.isFree(c); };
    if (!isFreeCell(start) || !isFreeCell(goal)) {
        throw std::invalid_argument(
            "spaceTimeAstar: start and goal must be free cells inside the grid");
    }
    if (static_cast<int>(h.size()) != n) {
        throw std::invalid_argument(
            "spaceTimeAstar: heuristic table size must equal grid.numCells()");
    }

    SearchStats local;
    SearchStats& s = stats != nullptr ? *stats : local;
    s = SearchStats{};

    // Automatic horizon. If any valid path exists, one exists that arrives by
    // maxTime + numCells: follow it until time maxTime + 1 (or until it finishes). From then on
    // nothing changes any more (only BlockedFrom cells stay blocked, and they are now fixed), so
    // a static shortest path around them takes < numCells moves.
    const int limit = horizon >= 0 ? horizon : constraints.maxTime() + n;

    // The earliest time the agent may finish. Using it in f keeps f admissible (the arrival
    // time is at least this) and stops the search from trying to finish too early.
    const int earliestFinish = constraints.latestVertexConstraintAt(goal) + 1;
    auto fValue = [&](CellId c, int t) { return std::max(t + h[c], earliestFinish); };

    if (h[start] == kUnreachable || constraints.vertexBlocked(start, 0) ||
        constraints.blockedFromAt(goal) >= 0 || fValue(start, 0) > limit) {
        // (a goal that gets blocked forever can never be the place the agent stays)
        return std::nullopt;
    }

    std::vector<Node> pool;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, PopsLater> open;
    std::unordered_set<std::uint64_t> seen;  // states already generated

    pool.push_back(Node{start, 0, -1});
    seen.insert(stateKey(start, 0));
    open.push(OpenEntry{fValue(start, 0), 0, start, 0});
    ++s.generated;

    std::vector<CellId> next;
    while (!open.empty()) {
        const OpenEntry e = open.top();
        open.pop();
        ++s.expanded;
        const Node node = pool[static_cast<std::size_t>(e.index)];  // copy: pool may grow below
        if (node.cell == goal && node.t >= earliestFinish) {
            return reconstruct(pool, e.index);
        }

        grid.neighbors(node.cell, next);
        next.push_back(node.cell);  // wait in place
        const int t = node.t + 1;
        for (CellId c : next) {
            if (h[c] == kUnreachable) continue;
            if (constraints.vertexBlocked(c, t)) continue;
            if (c != node.cell && constraints.edgeBlocked(node.cell, c, t)) continue;
            const int f = fValue(c, t);
            if (f > limit) continue;  // cannot finish by the horizon from here
            if (!seen.insert(stateKey(c, t)).second) continue;  // same state, same cost
            pool.push_back(Node{c, t, e.index});
            open.push(OpenEntry{f, t, c, static_cast<int>(pool.size()) - 1});
            ++s.generated;
        }
    }
    return std::nullopt;
}

}  // namespace swarmgrid

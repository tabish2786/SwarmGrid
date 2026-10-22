#include "swarmgrid/astar.hpp"

#include <algorithm>
#include <cstddef>
#include <queue>
#include <stdexcept>

#include "swarmgrid/heuristic.hpp"

namespace swarmgrid {

namespace {

struct Node {
    int f;
    int g;
    CellId cell;
};

// std::priority_queue pops the element that compares LARGEST, so this returns true
// when a should come out AFTER b.
struct PopsLater {
    bool operator()(const Node& a, const Node& b) const {
        if (a.f != b.f) return a.f > b.f;  // smaller f first
        if (a.g != b.g) return a.g < b.g;  // then larger g (closer to the goal)
        return a.cell > b.cell;            // then smaller cell id (determinism)
    }
};

Path reconstruct(const std::vector<CellId>& parent, CellId goal) {
    Path path;
    for (CellId c = goal; c != kInvalidCell; c = parent[c]) {
        path.push_back(c);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace

std::optional<Path> astar(const Grid& grid, CellId start, CellId goal, const std::vector<int>& h,
                          SearchStats* stats) {
    const int n = grid.numCells();
    auto isFreeCell = [&](CellId c) { return c >= 0 && c < n && grid.isFree(c); };
    if (!isFreeCell(start) || !isFreeCell(goal)) {
        throw std::invalid_argument("astar: start and goal must be free cells inside the grid");
    }
    if (static_cast<int>(h.size()) != n) {
        throw std::invalid_argument("astar: heuristic table size must equal grid.numCells()");
    }

    SearchStats local;
    SearchStats& s = stats != nullptr ? *stats : local;
    s = SearchStats{};

    if (h[start] == kUnreachable) {
        return std::nullopt;  // BFS heuristic already proved there is no path
    }

    std::vector<int> bestG(static_cast<std::size_t>(n), kUnreachable);
    std::vector<CellId> parent(static_cast<std::size_t>(n), kInvalidCell);
    std::priority_queue<Node, std::vector<Node>, PopsLater> open;

    bestG[start] = 0;
    open.push(Node{h[start], 0, start});
    ++s.generated;

    std::vector<CellId> next;
    while (!open.empty()) {
        const Node node = open.top();
        open.pop();
        // No decrease-key in std::priority_queue: we push duplicates instead and skip the
        // stale ones here ("lazy deletion").
        if (node.g > bestG[node.cell]) {
            continue;
        }
        ++s.expanded;
        if (node.cell == goal) {
            return reconstruct(parent, goal);
        }

        grid.neighbors(node.cell, next);
        for (CellId c : next) {
            if (h[c] == kUnreachable) {
                continue;
            }
            const int g = node.g + 1;
            if (g < bestG[c]) {
                bestG[c] = g;
                parent[c] = node.cell;
                open.push(Node{g + h[c], g, c});
                ++s.generated;
            }
        }
    }
    return std::nullopt;
}

}  // namespace swarmgrid

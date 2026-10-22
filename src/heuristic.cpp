#include "swarmgrid/heuristic.hpp"

#include <cstdlib>
#include <queue>
#include <stdexcept>
#include <string>

namespace swarmgrid {

namespace {

void requireFreeGoal(const Grid& grid, CellId goal, const char* who) {
    if (goal < 0 || goal >= grid.numCells() || !grid.isFree(goal)) {
        throw std::invalid_argument(std::string(who) + ": goal must be a free cell inside the grid");
    }
}

}  // namespace

std::vector<int> bfsDistances(const Grid& grid, CellId goal) {
    requireFreeGoal(grid, goal, "bfsDistances");

    std::vector<int> dist(static_cast<std::size_t>(grid.numCells()), kUnreachable);
    std::queue<CellId> frontier;
    dist[goal] = 0;
    frontier.push(goal);

    std::vector<CellId> next;  // reused across iterations: no allocation per cell
    while (!frontier.empty()) {
        const CellId c = frontier.front();
        frontier.pop();
        grid.neighbors(c, next);
        for (CellId n : next) {
            if (dist[n] == kUnreachable) {  // first visit is the shortest in BFS
                dist[n] = dist[c] + 1;
                frontier.push(n);
            }
        }
    }
    return dist;
}

std::vector<int> manhattanDistances(const Grid& grid, CellId goal) {
    requireFreeGoal(grid, goal, "manhattanDistances");

    const Point g = grid.toPoint(goal);
    std::vector<int> dist(static_cast<std::size_t>(grid.numCells()), kUnreachable);
    for (CellId c = 0; c < grid.numCells(); ++c) {
        if (grid.isFree(c)) {
            const Point p = grid.toPoint(c);
            dist[c] = std::abs(p.x - g.x) + std::abs(p.y - g.y);
        }
    }
    return dist;
}

}  // namespace swarmgrid

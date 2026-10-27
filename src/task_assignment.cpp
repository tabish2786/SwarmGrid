#include "swarmgrid/task_assignment.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include "swarmgrid/heuristic.hpp"

namespace swarmgrid {

namespace {

// Checks the cells, then returns dist[o][c] = moves from cell c to order o. BFS from the order
// works because moves are undirected (same trick as the A* heuristic).
std::vector<std::vector<int>> orderDistances(const Grid& grid, const std::vector<CellId>& robotCells,
                                             const std::vector<CellId>& orderCells, const char* who) {
    auto requireFree = [&](CellId c, const char* what) {
        if (c < 0 || c >= grid.numCells() || !grid.isFree(c)) {
            throw std::invalid_argument(std::string(who) + ": a " + what + " is not on a free cell");
        }
    };
    for (CellId c : robotCells) requireFree(c, "robot");
    for (CellId c : orderCells) requireFree(c, "order");
    std::vector<std::vector<int>> dist;
    dist.reserve(orderCells.size());
    for (CellId c : orderCells) dist.push_back(bfsDistances(grid, c));
    return dist;
}

}  // namespace

std::vector<Assignment> assignNearest(const Grid& grid, const std::vector<CellId>& robotCells,
                                      const std::vector<CellId>& orderCells) {
    const auto dist = orderDistances(grid, robotCells, orderCells, "assignNearest");

    std::vector<bool> robotTaken(robotCells.size(), false);
    std::vector<bool> orderTaken(orderCells.size(), false);
    std::vector<Assignment> result;
    while (true) {
        Assignment best;
        best.distance = kUnreachable;
        for (std::size_t o = 0; o < orderCells.size(); ++o) {
            if (orderTaken[o]) continue;
            for (std::size_t r = 0; r < robotCells.size(); ++r) {
                if (robotTaken[r]) continue;
                const int d = dist[o][static_cast<std::size_t>(robotCells[r])];
                // Strictly smaller only: the loop order already prefers smaller indices on ties.
                if (d < best.distance) {
                    best.robot = static_cast<int>(r);
                    best.order = static_cast<int>(o);
                    best.distance = d;
                }
            }
        }
        if (best.robot < 0) break;  // nobody free can reach anything free
        robotTaken[static_cast<std::size_t>(best.robot)] = true;
        orderTaken[static_cast<std::size_t>(best.order)] = true;
        result.push_back(best);
    }
    return result;
}

std::vector<Assignment> assignOptimal(const Grid& grid, const std::vector<CellId>& robotCells,
                                      const std::vector<CellId>& orderCells) {
    const auto dist = orderDistances(grid, robotCells, orderCells, "assignOptimal");
    const int k = static_cast<int>(robotCells.size());
    const int m = static_cast<int>(orderCells.size());
    if (k == 0 || m == 0) return {};

    // Square n x n matrix: row = robot, column = order, dummies cost 0. An unreachable real pair
    // costs `big`, more than any set of real distances together, so the optimum first avoids
    // unreachable pairs (= assigns as many reachable pairs as possible), then minimises distance.
    const int n = std::max(k, m);
    const long long big = static_cast<long long>(grid.numCells()) * std::min(k, m) + 1;
    auto cost = [&](int r, int o) -> long long {  // 1-based, as in the algorithm below
        if (r > k || o > m) return 0;
        const int d = dist[static_cast<std::size_t>(o - 1)][static_cast<std::size_t>(robotCells[static_cast<std::size_t>(r - 1)])];
        return d == kUnreachable ? big : d;
    };

    // Potentials u (rows) and v (columns) keep cost(r, o) - u[r] - v[o] >= 0, with equality on
    // matched pairs. Rows are added one at a time; each addition grows a shortest augmenting path
    // (Dijkstra on the reduced costs) from the new row to a free column, then flips it.
    // p[o] = row matched to column o (0 = none); index 0 is the virtual start column.
    const long long inf = (std::numeric_limits<long long>::max)() / 4;
    std::vector<long long> u(static_cast<std::size_t>(n) + 1, 0), v(static_cast<std::size_t>(n) + 1, 0);
    std::vector<int> p(static_cast<std::size_t>(n) + 1, 0), way(static_cast<std::size_t>(n) + 1, 0);
    for (int row = 1; row <= n; ++row) {
        p[0] = row;
        int col0 = 0;
        std::vector<long long> minv(static_cast<std::size_t>(n) + 1, inf);
        std::vector<char> used(static_cast<std::size_t>(n) + 1, 0);
        do {
            used[static_cast<std::size_t>(col0)] = 1;
            const int row0 = p[static_cast<std::size_t>(col0)];
            long long delta = inf;
            int col1 = 0;
            for (int col = 1; col <= n; ++col) {
                const auto c = static_cast<std::size_t>(col);
                if (used[c]) continue;
                const long long reduced = cost(row0, col) - u[static_cast<std::size_t>(row0)] - v[c];
                if (reduced < minv[c]) {
                    minv[c] = reduced;
                    way[c] = col0;
                }
                if (minv[c] < delta) {  // strict: the smallest column index wins ties
                    delta = minv[c];
                    col1 = col;
                }
            }
            for (int col = 0; col <= n; ++col) {
                const auto c = static_cast<std::size_t>(col);
                if (used[c]) {
                    u[static_cast<std::size_t>(p[c])] += delta;
                    v[c] -= delta;
                } else {
                    minv[c] -= delta;
                }
            }
            col0 = col1;
        } while (p[static_cast<std::size_t>(col0)] != 0);
        do {  // flip the augmenting path
            const int col1 = way[static_cast<std::size_t>(col0)];
            p[static_cast<std::size_t>(col0)] = p[static_cast<std::size_t>(col1)];
            col0 = col1;
        } while (col0 != 0);
    }

    std::vector<Assignment> result;
    for (int o = 1; o <= m; ++o) {
        const int r = p[static_cast<std::size_t>(o)];
        if (r < 1 || r > k) continue;  // matched to a dummy robot: this order waits
        const int d = dist[static_cast<std::size_t>(o - 1)][static_cast<std::size_t>(robotCells[static_cast<std::size_t>(r - 1)])];
        if (d == kUnreachable) continue;
        result.push_back(Assignment{r - 1, o - 1, d});
    }
    return result;
}

}  // namespace swarmgrid

#include "doctest/doctest.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "swarmgrid/astar.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid::kUnreachable;
using swarmgrid::Path;
using swarmgrid::SearchStats;

namespace {

//   . . @ . .
//   . . @ . .
//   . . . . .
Grid exampleMap() {
    Grid g(5, 3);
    g.setObstacle(2, 0, true);
    g.setObstacle(2, 1, true);
    return g;
}

// Starts at s, ends at t, every cell free, every step moves exactly one cell (no waits in M1).
bool isValidPath(const Grid& g, const Path& p, CellId s, CellId t) {
    if (p.empty() || p.front() != s || p.back() != t) return false;
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (!g.isFree(p[i])) return false;
        if (i > 0) {
            const swarmgrid::Point a = g.toPoint(p[i - 1]);
            const swarmgrid::Point b = g.toPoint(p[i]);
            if (std::abs(a.x - b.x) + std::abs(a.y - b.y) != 1) return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("astar: worked example, exact path and stats (pins the tie-breaking)") {
    const Grid g = exampleMap();
    const CellId s = g.toId(0, 0);
    const CellId t = g.toId(4, 0);
    // Several 8-move paths exist. The tie-break (larger g, then smaller cell id) picks this one.
    const Path expected{g.toId(0, 0), g.toId(1, 0), g.toId(1, 1), g.toId(1, 2), g.toId(2, 2),
                        g.toId(3, 2), g.toId(3, 1), g.toId(3, 0), g.toId(4, 0)};

    SUBCASE("BFS heuristic") {
        SearchStats stats;
        const auto path = swarmgrid::astar(g, s, t, swarmgrid::bfsDistances(g, t), &stats);
        REQUIRE(path.has_value());
        CHECK(*path == expected);
        CHECK(isValidPath(g, *path, s, t));
        CHECK(stats.expanded == 9);
        CHECK(stats.generated == 13);
    }
    SUBCASE("Manhattan heuristic") {
        SearchStats stats;
        const auto path = swarmgrid::astar(g, s, t, swarmgrid::manhattanDistances(g, t), &stats);
        REQUIRE(path.has_value());
        CHECK(*path == expected);
        CHECK(stats.expanded == 10);
        CHECK(stats.generated == 13);
    }
}

TEST_CASE("astar: exact path on the warehouse (cross-platform determinism)") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const CellId s = g.toId(0, 0);
    const CellId t = g.toId(13, 10);
    REQUIRE(g.isFree(t));
    Path expected;
    for (int x = 0; x <= 13; ++x) expected.push_back(g.toId(x, 0));   // along row 0
    for (int y = 1; y <= 10; ++y) expected.push_back(g.toId(13, y));  // down aisle x = 13

    SearchStats stats;
    const auto path = swarmgrid::astar(g, s, t, swarmgrid::manhattanDistances(g, t), &stats);
    REQUIRE(path.has_value());
    CHECK(*path == expected);
    CHECK(stats.expanded == 24);
    CHECK(stats.generated == 43);
}

TEST_CASE("astar: perfect heuristic expands only the cells on the returned path") {
    // With exact distances, every node on SOME optimal path has f == C* (often many nodes), all
    // others have f > C*. The larger-g tie-break always pops the deepest f == C* node, so the
    // search walks one optimal path: expanded == C* + 1 == path.size(). This is guaranteed by
    // the comparator and breaks if the g tie-break is removed or reversed.
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const CellId s = g.toId(0, 0);
    const CellId t = g.toId(g.width() - 1, g.height() - 1);
    SearchStats stats;
    const auto path = swarmgrid::astar(g, s, t, swarmgrid::bfsDistances(g, t), &stats);
    REQUIRE(path.has_value());
    CHECK(stats.expanded == static_cast<int>(path->size()));

    SearchStats manStats;
    const auto manPath = swarmgrid::astar(g, s, t, swarmgrid::manhattanDistances(g, t), &manStats);
    REQUIRE(manPath.has_value());
    CHECK(manPath->size() == path->size());
    CHECK(manStats.expanded >= stats.expanded);
}

TEST_CASE("astar: start == goal") {
    const Grid g = exampleMap();
    const CellId s = g.toId(1, 1);
    SearchStats stats;
    const auto path = swarmgrid::astar(g, s, s, swarmgrid::bfsDistances(g, s), &stats);
    REQUIRE(path.has_value());
    CHECK(*path == Path{s});
    CHECK(stats.expanded == 1);
    CHECK(stats.generated == 1);
}

TEST_CASE("astar: unreachable goal") {
    SUBCASE("BFS heuristic proves it without searching") {
        //   . @ .
        //   @ @ .
        //   . . .
        Grid g(3, 3);
        g.setObstacle(1, 0, true);
        g.setObstacle(0, 1, true);
        g.setObstacle(1, 1, true);
        const CellId t = g.toId(2, 2);
        SearchStats stats;
        CHECK_FALSE(
            swarmgrid::astar(g, g.toId(0, 0), t, swarmgrid::bfsDistances(g, t), &stats).has_value());
        CHECK(stats.expanded == 0);
    }
    SUBCASE("Manhattan exhausts the reachable region and skips a stale duplicate") {
        //   S .      (0,2) is first pushed with g = 4 via (1,2), then improved to g = 2 via
        //   . .      (0,1). The stale g = 4 entry pops last and must be skipped, not expanded.
        //   . .
        //   @ @      row 3 is a wall, so the goal below it is unreachable
        //   . G
        Grid g(2, 5);
        g.setObstacle(0, 3, true);
        g.setObstacle(1, 3, true);
        const CellId t = g.toId(1, 4);
        SearchStats stats;
        CHECK_FALSE(swarmgrid::astar(g, g.toId(0, 0), t, swarmgrid::manhattanDistances(g, t), &stats)
                        .has_value());
        CHECK(stats.expanded == 6);  // all 6 reachable cells, each exactly once
        CHECK(stats.generated == 7);
    }
}

TEST_CASE("astar: custom tables honour kUnreachable pruning") {
    const Grid g = exampleMap();
    const CellId s = g.toId(0, 0);
    const CellId t = g.toId(4, 0);

    SUBCASE("pruning the only gap makes the goal unreachable") {
        std::vector<int> h = swarmgrid::manhattanDistances(g, t);
        h[g.toId(2, 2)] = kUnreachable;  // without the prune, g + INT_MAX would overflow
        CHECK_FALSE(swarmgrid::astar(g, s, t, h).has_value());
    }
    SUBCASE("a pruned cell is avoided, path stays optimal") {
        std::vector<int> h = swarmgrid::bfsDistances(g, t);
        h[g.toId(1, 0)] = kUnreachable;
        const auto path = swarmgrid::astar(g, s, t, h);
        REQUIRE(path.has_value());
        CHECK(path->size() == 9);
        CHECK(isValidPath(g, *path, s, t));
        CHECK(std::find(path->begin(), path->end(), g.toId(1, 0)) == path->end());
    }
}

TEST_CASE("astar: admissible but inconsistent heuristic needs re-expansion to stay optimal") {
    //   S A X . . . . G
    //   B @ Y @ @ @ @ @
    //   C D E @ @ @ @ @
    // h = 6 at A (its true distance), 0 everywhere else: admissible, not consistent. The detour
    // S-B-C-D-E-Y reaches X first with g = 6; A pops later and X must be re-opened with g = 2.
    // A "closed set" that never re-opens would return 11 moves instead of 7.
    Grid g(8, 3);
    g.setObstacle(1, 1, true);
    for (int x = 3; x < 8; ++x) {
        g.setObstacle(x, 1, true);
        g.setObstacle(x, 2, true);
    }
    std::vector<int> h(static_cast<std::size_t>(g.numCells()), 0);
    h[g.toId(1, 0)] = 6;

    SearchStats stats;
    const auto path = swarmgrid::astar(g, g.toId(0, 0), g.toId(7, 0), h, &stats);
    REQUIRE(path.has_value());
    CHECK(path->size() == 8);
    CHECK(stats.expanded == 16);  // more than the 13 free cells: re-expansions are counted
    CHECK(stats.generated == 17);
}

TEST_CASE("astar: stats object is reset on every call") {
    const Grid g = exampleMap();
    const CellId t = g.toId(4, 0);
    const auto h = swarmgrid::bfsDistances(g, t);
    SearchStats stats;
    (void)swarmgrid::astar(g, g.toId(0, 0), t, h, &stats);
    (void)swarmgrid::astar(g, g.toId(0, 0), t, h, &stats);
    CHECK(stats.expanded == 9);
    CHECK(stats.generated == 13);
}

TEST_CASE("astar: optimal on random warehouse queries (cross-checked against BFS)") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 50, 7);
    for (int i = 0; i < inst.numAgents(); ++i) {
        const CellId s = g.toId(inst.starts[static_cast<std::size_t>(i)]);
        const CellId t = g.toId(inst.goals[static_cast<std::size_t>(i)]);
        const auto dist = swarmgrid::bfsDistances(g, t);

        SearchStats bfsStats;
        SearchStats manStats;
        const auto p1 = swarmgrid::astar(g, s, t, dist, &bfsStats);
        const auto p2 = swarmgrid::astar(g, s, t, swarmgrid::manhattanDistances(g, t), &manStats);
        REQUIRE(p1.has_value());
        REQUIRE(p2.has_value());
        CHECK(static_cast<int>(p1->size()) - 1 == dist[s]);
        CHECK(p2->size() == p1->size());
        CHECK(isValidPath(g, *p1, s, t));
        CHECK(isValidPath(g, *p2, s, t));
        CHECK(bfsStats.expanded == static_cast<int>(p1->size()));
        CHECK(manStats.expanded >= bfsStats.expanded);
    }
}

TEST_CASE("astar: golden totals for 'swarmgrid astar-bench 100 42'") {
    // Same numbers the CLI prints. They must match on Linux, MinGW and MSVC; if they ever
    // differ, determinism is broken somewhere (RNG, tie-breaking, neighbor order).
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 100, 42);
    long long moves = 0;
    long long bfsExpanded = 0;
    long long manExpanded = 0;
    for (int i = 0; i < inst.numAgents(); ++i) {
        const CellId s = g.toId(inst.starts[static_cast<std::size_t>(i)]);
        const CellId t = g.toId(inst.goals[static_cast<std::size_t>(i)]);
        SearchStats a;
        SearchStats b;
        const auto p = swarmgrid::astar(g, s, t, swarmgrid::bfsDistances(g, t), &a);
        (void)swarmgrid::astar(g, s, t, swarmgrid::manhattanDistances(g, t), &b);
        REQUIRE(p.has_value());
        moves += static_cast<long long>(p->size()) - 1;
        bfsExpanded += a.expanded;
        manExpanded += b.expanded;
    }
    CHECK(moves == 1857);
    CHECK(bfsExpanded == 1957);
    CHECK(manExpanded == 2162);
}

TEST_CASE("astar: invalid inputs throw") {
    const Grid g = exampleMap();
    const CellId t = g.toId(4, 0);
    const auto h = swarmgrid::bfsDistances(g, t);
    CHECK_THROWS_AS(swarmgrid::astar(g, g.toId(2, 0), t, h), std::invalid_argument);  // start on shelf
    CHECK_THROWS_AS(swarmgrid::astar(g, g.toId(0, 0), g.toId(2, 0), h),
                    std::invalid_argument);  // goal on shelf
    CHECK_THROWS_AS(swarmgrid::astar(g, -1, t, h), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::astar(g, g.numCells(), t, h), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::astar(g, g.toId(0, 0), g.numCells(), h), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::astar(g, g.toId(0, 0), t, std::vector<int>(3, 0)),
                    std::invalid_argument);
}

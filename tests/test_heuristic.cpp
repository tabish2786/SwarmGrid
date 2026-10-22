#include "doctest/doctest.h"

#include <stdexcept>
#include <vector>

#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid::kUnreachable;

namespace {

// The M0/M1 worked example:
//   . . @ . .
//   . . @ . .
//   . . . . .
Grid exampleMap() {
    Grid g(5, 3);
    g.setObstacle(2, 0, true);
    g.setObstacle(2, 1, true);
    return g;
}

}  // namespace

TEST_CASE("bfs: matches the worked example table") {
    const Grid g = exampleMap();
    const auto d = swarmgrid::bfsDistances(g, g.toId(4, 0));
    const int X = kUnreachable;
    const std::vector<int> expected{
        8, 7, X, 1, 0,
        7, 6, X, 2, 1,
        6, 5, 4, 3, 2,
    };
    CHECK(d == expected);
}

TEST_CASE("bfs: walled-off cells are unreachable") {
    //   . @ .
    //   @ @ .
    //   . . .
    Grid g(3, 3);
    g.setObstacle(1, 0, true);
    g.setObstacle(0, 1, true);
    g.setObstacle(1, 1, true);
    const auto d = swarmgrid::bfsDistances(g, g.toId(2, 2));
    CHECK(d[g.toId(0, 0)] == kUnreachable);
    CHECK(d[g.toId(2, 0)] == 2);
    CHECK(d[g.toId(0, 2)] == 2);
    CHECK(d[g.toId(1, 1)] == kUnreachable);  // obstacle
}

TEST_CASE("bfs / manhattan: goal must be a free cell inside the grid") {
    const Grid g = exampleMap();
    CHECK_THROWS_AS(swarmgrid::bfsDistances(g, g.toId(2, 0)), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::bfsDistances(g, -1), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::bfsDistances(g, g.numCells()), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::manhattanDistances(g, g.toId(2, 1)), std::invalid_argument);
}

TEST_CASE("manhattan: matches the hand-computed table on the worked example") {
    const Grid g = exampleMap();
    const auto d = swarmgrid::manhattanDistances(g, g.toId(4, 0));
    const int X = kUnreachable;
    const std::vector<int> expected{
        4, 3, X, 1, 0,
        5, 4, X, 2, 1,
        6, 5, 4, 3, 2,
    };
    CHECK(d == expected);
}

TEST_CASE("manhattan: exact values and admissible against BFS on the warehouse") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    // Goal in the aisle between the first two shelf blocks. (13,2) sits in the next aisle on
    // the same row: 6 cells away as the crow flies, but the shelf block in between forces a
    // detour through row 1, so the true distance is 8.
    const CellId goal = g.toId(7, 2);
    REQUIRE(g.isFree(goal));  // these coordinates assume the default warehouse layout
    REQUIRE(g.isFree(g.toId(13, 2)));
    const auto bfs = swarmgrid::bfsDistances(g, goal);
    const auto man = swarmgrid::manhattanDistances(g, goal);
    CHECK(man[g.toId(13, 2)] == 6);
    CHECK(bfs[g.toId(13, 2)] == 8);

    int violations = 0;
    int strictlyBelow = 0;
    for (CellId c = 0; c < g.numCells(); ++c) {
        if (!g.isFree(c)) {
            if (man[c] != kUnreachable || bfs[c] != kUnreachable) ++violations;
            continue;
        }
        if (bfs[c] == kUnreachable || man[c] > bfs[c]) ++violations;
        if (man[c] < bfs[c]) ++strictlyBelow;
    }
    CHECK(violations == 0);
    CHECK(strictlyBelow > 0);  // shelves make Manhattan a real underestimate somewhere
}

#include "doctest/doctest.h"

#include <stdexcept>
#include <vector>

#include "swarmgrid/grid.hpp"

using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid::Point;

TEST_CASE("grid: constructor validates size") {
    CHECK_THROWS_AS(Grid(0, 5), std::invalid_argument);
    CHECK_THROWS_AS(Grid(5, 0), std::invalid_argument);
    CHECK_THROWS_AS(Grid(-1, 3), std::invalid_argument);

    const Grid g(7, 3);
    CHECK(g.width() == 7);
    CHECK(g.height() == 3);
    CHECK(g.numCells() == 21);
    CHECK(g.countObstacles() == 0);
}

TEST_CASE("grid: toId / toPoint round trip on a non-square grid") {
    // Non-square on purpose: a grid where width == height hides x/y swap bugs.
    const Grid g(7, 3);
    CHECK(g.toId(0, 0) == 0);
    CHECK(g.toId(6, 0) == 6);
    CHECK(g.toId(0, 1) == 7);
    CHECK(g.toId(6, 2) == 20);
    for (CellId c = 0; c < g.numCells(); ++c) {
        const Point p = g.toPoint(c);
        CHECK(g.toId(p) == c);
    }
    CHECK(g.toPoint(9) == Point{2, 1});
}

TEST_CASE("grid: bounds and free checks") {
    Grid g(4, 3);
    CHECK(g.inBounds(0, 0));
    CHECK(g.inBounds(3, 2));
    CHECK_FALSE(g.inBounds(4, 0));
    CHECK_FALSE(g.inBounds(0, 3));
    CHECK_FALSE(g.inBounds(-1, 0));
    CHECK_FALSE(g.inBounds(0, -1));

    g.setObstacle(1, 2, true);
    CHECK_FALSE(g.isFree(1, 2));
    CHECK_FALSE(g.isFree(g.toId(1, 2)));
    CHECK(g.isFree(2, 1));
    CHECK(g.countObstacles() == 1);

    g.setObstacle(1, 2, false);
    CHECK(g.isFree(1, 2));
    CHECK(g.countObstacles() == 0);

    // Out of bounds is "not free", never a crash.
    CHECK_FALSE(g.isFree(-1, 0));
    CHECK_FALSE(g.isFree(4, 0));

    CHECK_THROWS_AS(g.setObstacle(4, 0, true), std::out_of_range);
}

TEST_CASE("grid: neighbors respect obstacles, bounds, and the fixed order") {
    //   x: 0 1 2
    // y0:  . . .
    // y1:  . . @
    // y2:  . . .
    Grid g(3, 3);
    g.setObstacle(2, 1, true);
    std::vector<CellId> out;

    SUBCASE("center: up, down, left (right is a shelf)") {
        g.neighbors(g.toId(1, 1), out);
        const std::vector<CellId> expected{g.toId(1, 0), g.toId(1, 2), g.toId(0, 1)};
        CHECK(out == expected);
    }
    SUBCASE("top-left corner: down, right") {
        g.neighbors(g.toId(0, 0), out);
        const std::vector<CellId> expected{g.toId(0, 1), g.toId(1, 0)};
        CHECK(out == expected);
    }
    SUBCASE("bottom-right corner: left only (up is a shelf)") {
        g.neighbors(g.toId(2, 2), out);
        const std::vector<CellId> expected{g.toId(1, 2)};
        CHECK(out == expected);
    }
    SUBCASE("output buffer is cleared first") {
        out = {99, 98, 97, 96, 95};
        g.neighbors(g.toId(0, 0), out);
        CHECK(out.size() == 2);
    }
    SUBCASE("no wrap-around between rows") {
        // (2,0) is the end of row 0; id+1 is (0,1). It must NOT be a neighbour.
        g.neighbors(g.toId(2, 0), out);
        const std::vector<CellId> expected{g.toId(1, 0)};
        CHECK(out == expected);
    }
}

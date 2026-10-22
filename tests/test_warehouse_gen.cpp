#include "doctest/doctest.h"

#include <climits>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "swarmgrid/ascii.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::Grid;
using swarmgrid::Point;
using swarmgrid::Rng;
using swarmgrid::WarehouseParams;

TEST_CASE("warehouse: exact layout of the small example from the header") {
    WarehouseParams p;
    p.blocksX = 2;
    p.blocksY = 2;
    p.shelfBlockWidth = 3;
    p.shelfBlockHeight = 1;
    p.aisleWidth = 1;
    p.border = 1;
    const Grid g = swarmgrid::generateWarehouse(p);
    CHECK(swarmgrid::renderGrid(g) ==
          ".........\n"
          ".@@@.@@@.\n"
          ".........\n"
          ".@@@.@@@.\n"
          ".........\n");
}

TEST_CASE("warehouse: dimensions, shelf count, free border") {
    const WarehouseParams p;  // defaults
    const Grid g = swarmgrid::generateWarehouse(p);
    const int w = 2 * p.border + p.blocksX * p.shelfBlockWidth + (p.blocksX - 1) * p.aisleWidth;
    const int h = 2 * p.border + p.blocksY * p.shelfBlockHeight + (p.blocksY - 1) * p.aisleWidth;
    CHECK(g.width() == w);
    CHECK(g.height() == h);
    CHECK(g.countObstacles() ==
          p.blocksX * p.blocksY * p.shelfBlockWidth * p.shelfBlockHeight);
    for (int x = 0; x < g.width(); ++x) {
        for (int b = 0; b < p.border; ++b) {
            CHECK(g.isFree(x, b));
            CHECK(g.isFree(x, g.height() - 1 - b));
        }
    }
    for (int y = 0; y < g.height(); ++y) {
        for (int b = 0; b < p.border; ++b) {
            CHECK(g.isFree(b, y));
            CHECK(g.isFree(g.width() - 1 - b, y));
        }
    }
}

TEST_CASE("warehouse: invalid params throw") {
    WarehouseParams p;
    p.aisleWidth = 0;
    CHECK_THROWS_AS(swarmgrid::generateWarehouse(p), std::invalid_argument);
    p = WarehouseParams{};
    p.blocksX = 0;
    CHECK_THROWS_AS(swarmgrid::generateWarehouse(p), std::invalid_argument);
    p = WarehouseParams{};
    p.border = -1;
    CHECK_THROWS_AS(swarmgrid::generateWarehouse(p), std::invalid_argument);
}

TEST_CASE("rng: mt19937 is the standard-mandated engine") {
    // C++ standard [rand.predef]: the 10000th output of a default-constructed
    // std::mt19937 (seed 5489) is 4123659995. Same on MSVC, GCC, Clang.
    Rng r(5489u);
    std::uint32_t v = 0;
    for (int i = 0; i < 10000; ++i) v = r.next();
    CHECK(v == 4123659995u);
}

TEST_CASE("rng: uniformInt range, determinism, rough uniformity") {
    CHECK_THROWS_AS(Rng(1).uniformInt(5, 4), std::invalid_argument);
    CHECK(Rng(1).uniformInt(7, 7) == 7);

    Rng a(123), b(123);
    for (int i = 0; i < 1000; ++i) {
        const int x = a.uniformInt(-3, 9);
        CHECK(x >= -3);
        CHECK(x <= 9);
        CHECK(x == b.uniformInt(-3, 9));
    }

    // Full int range must not overflow or crash.
    Rng full(7);
    for (int i = 0; i < 100; ++i) (void)full.uniformInt(INT_MIN, INT_MAX);

    // 60000 draws over 6 buckets: each should be near 10000. Loose bound, not a stats test.
    Rng r(2024);
    std::vector<int> counts(6, 0);
    for (int i = 0; i < 60000; ++i) ++counts[static_cast<std::size_t>(r.uniformInt(0, 5))];
    for (int c : counts) {
        CHECK(c > 9000);
        CHECK(c < 11000);
    }
}

TEST_CASE("random instance: valid, distinct, deterministic") {
    const Grid g = swarmgrid::generateWarehouse(WarehouseParams{});
    const int k = 50;
    const auto inst = swarmgrid::generateRandomInstance(g, k, 42);
    REQUIRE(inst.numAgents() == k);
    REQUIRE(inst.goals.size() == static_cast<std::size_t>(k));

    std::set<std::pair<int, int>> starts, goals;
    for (int i = 0; i < k; ++i) {
        CHECK(g.isFree(inst.starts[i].x, inst.starts[i].y));
        CHECK(g.isFree(inst.goals[i].x, inst.goals[i].y));
        starts.insert({inst.starts[i].x, inst.starts[i].y});
        goals.insert({inst.goals[i].x, inst.goals[i].y});
    }
    CHECK(starts.size() == static_cast<std::size_t>(k));
    CHECK(goals.size() == static_cast<std::size_t>(k));

    const auto again = swarmgrid::generateRandomInstance(g, k, 42);
    CHECK(again.starts == inst.starts);
    CHECK(again.goals == inst.goals);

    const auto other = swarmgrid::generateRandomInstance(g, k, 43);
    CHECK(other.starts != inst.starts);

    CHECK_THROWS_AS(swarmgrid::generateRandomInstance(g, 0, 1), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::generateRandomInstance(g, g.numCells(), 1),
                    std::invalid_argument);
}

TEST_CASE("random instance: k == number of free cells works") {
    Grid g(3, 2);
    g.setObstacle(1, 0, true);
    const auto inst = swarmgrid::generateRandomInstance(g, 5, 9);
    CHECK(inst.numAgents() == 5);
}

#include "doctest/doctest.h"

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/task_assignment.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::Assignment;
using swarmgrid::CellId;
using swarmgrid::Grid;

namespace {

int totalDistance(const std::vector<Assignment>& a) {
    int s = 0;
    for (const Assignment& x : a) s += x.distance;
    return s;
}

}  // namespace

TEST_CASE("assignment: closest pairs first") {
    const Grid g(10, 1);  // a corridor: cell id == x
    const std::vector<CellId> robots{0, 9};
    const std::vector<CellId> orders{8, 1};
    const auto a = swarmgrid::assignNearest(g, robots, orders);
    REQUIRE(a.size() == 2);
    // Both pairs have distance 1; the tie goes to the smaller order index first.
    CHECK(a[0].order == 0);
    CHECK(a[0].robot == 1);
    CHECK(a[0].distance == 1);
    CHECK(a[1].order == 1);
    CHECK(a[1].robot == 0);
    CHECK(a[1].distance == 1);
}

TEST_CASE("assignment: distance goes around shelves, not through them") {
    //   R . @ O       3 cells away in a straight line, 7 moves around the shelf
    //   . . @ .
    //   . . . .
    Grid g(4, 3);
    g.setObstacle(2, 0, true);
    g.setObstacle(2, 1, true);
    const auto a = swarmgrid::assignNearest(g, {g.toId(0, 0)}, {g.toId(3, 0)});
    REQUIRE(a.size() == 1);
    CHECK(a[0].distance == 7);
}

TEST_CASE("assignment: greedy is not optimal (documented limitation)") {
    // Robots at x=0 and x=3, orders at x=2 and x=5. Greedy grabs the 1-step pair (3 -> 2) first
    // and leaves 0 -> 5 (5 steps): total 6. The optimum (0 -> 2, 3 -> 5) totals 4.
    const Grid g(6, 1);
    const auto a = swarmgrid::assignNearest(g, {0, 3}, {2, 5});
    REQUIRE(a.size() == 2);
    CHECK(totalDistance(a) == 6);
}

TEST_CASE("assignment: more robots than orders, more orders than robots, unreachable orders") {
    const Grid g(10, 1);
    SUBCASE("extra robots stay idle") {
        const auto a = swarmgrid::assignNearest(g, {0, 4, 9}, {5});
        REQUIRE(a.size() == 1);
        CHECK(a[0].robot == 1);
    }
    SUBCASE("extra orders wait") {
        const auto a = swarmgrid::assignNearest(g, {4}, {0, 5, 9});
        REQUIRE(a.size() == 1);
        CHECK(a[0].order == 1);
    }
    SUBCASE("an order behind a wall is never assigned") {
        Grid w(5, 1);
        w.setObstacle(2, 0, true);
        const auto a = swarmgrid::assignNearest(w, {0, 1}, {4, 1});
        REQUIRE(a.size() == 1);
        CHECK(a[0].order == 1);
        CHECK(a[0].robot == 1);  // the robot standing on the order, distance 0
        CHECK(a[0].distance == 0);
    }
    SUBCASE("nothing to do") {
        CHECK(swarmgrid::assignNearest(g, {}, {3}).empty());
        CHECK(swarmgrid::assignNearest(g, {3}, {}).empty());
    }
}

TEST_CASE("assignment: invalid cells throw") {
    Grid g(3, 1);
    g.setObstacle(1, 0, true);
    CHECK_THROWS_AS(swarmgrid::assignNearest(g, {1}, {0}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::assignNearest(g, {0}, {7}), std::invalid_argument);
}

// ---- M8 support: optimal (Hungarian) assignment ----

namespace {

// Brute force over every matching: most reachable pairs first, then smallest total distance.
std::pair<int, int> bestMatching(const Grid& g, const std::vector<CellId>& robots,
                                 const std::vector<CellId>& orders) {
    std::vector<std::vector<int>> dist;
    for (CellId o : orders) dist.push_back(swarmgrid::bfsDistances(g, o));
    std::pair<int, int> best{0, 0};  // (pairs, -total) maximised
    std::vector<bool> used(robots.size(), false);
    std::function<void(std::size_t, int, int)> go = [&](std::size_t o, int pairs, int total) {
        if (o == orders.size()) {
            if (pairs > best.first || (pairs == best.first && -total > best.second)) best = {pairs, -total};
            return;
        }
        go(o + 1, pairs, total);  // this order waits
        for (std::size_t r = 0; r < robots.size(); ++r) {
            const int d = dist[o][static_cast<std::size_t>(robots[r])];
            if (used[r] || d == swarmgrid::kUnreachable) continue;
            used[r] = true;
            go(o + 1, pairs + 1, total + d);
            used[r] = false;
        }
    };
    go(0, 0, 0);
    return {best.first, -best.second};
}

}  // namespace

TEST_CASE("hungarian: beats greedy on the documented example") {
    const Grid g(6, 1);
    const auto a = swarmgrid::assignOptimal(g, {0, 3}, {2, 5});
    REQUIRE(a.size() == 2);
    CHECK(totalDistance(a) == 4);  // greedy: 6
    CHECK(a[0].order == 0);
    CHECK(a[0].robot == 0);
    CHECK(a[1].order == 1);
    CHECK(a[1].robot == 1);
}

TEST_CASE("hungarian: rectangular, unreachable pairs, empty input") {
    const Grid g(10, 1);
    CHECK(swarmgrid::assignOptimal(g, {}, {3}).empty());
    CHECK(swarmgrid::assignOptimal(g, {3}, {}).empty());
    const auto extraRobots = swarmgrid::assignOptimal(g, {0, 4, 9}, {5});
    REQUIRE(extraRobots.size() == 1);
    CHECK(extraRobots[0].robot == 1);
    const auto extraOrders = swarmgrid::assignOptimal(g, {4}, {0, 5, 9});
    REQUIRE(extraOrders.size() == 1);
    CHECK(extraOrders[0].order == 1);
    Grid w(5, 1);
    w.setObstacle(2, 0, true);
    // Robot 0 can only reach order 1; robot 1 (right side) only order 0. Both get one.
    const auto split = swarmgrid::assignOptimal(w, {0, 4}, {3, 1});
    REQUIRE(split.size() == 2);
    CHECK(split[0].robot == 1);
    CHECK(split[1].robot == 0);
    CHECK_THROWS_AS(swarmgrid::assignOptimal(w, {2}, {0}), std::invalid_argument);
}

TEST_CASE("hungarian: matches brute force on random small warehouses") {
    swarmgrid::Rng rng(99);
    for (int trial = 0; trial < 200; ++trial) {
        Grid g(6, 5);
        for (int i = rng.uniformInt(0, 8); i > 0; --i) {
            const int x = rng.uniformInt(0, 5);
            const int y = rng.uniformInt(0, 4);
            g.setObstacle(x, y, true);
        }
        std::vector<CellId> free;
        for (CellId c = 0; c < g.numCells(); ++c) {
            if (g.isFree(c)) free.push_back(c);
        }
        const int k = rng.uniformInt(0, 5), m = rng.uniformInt(0, 5);
        std::vector<CellId> robots, orders;
        for (int i = 0; i < k; ++i) robots.push_back(free[static_cast<std::size_t>(rng.uniformInt(0, static_cast<int>(free.size()) - 1))]);
        for (int i = 0; i < m; ++i) orders.push_back(free[static_cast<std::size_t>(rng.uniformInt(0, static_cast<int>(free.size()) - 1))]);
        const auto a = swarmgrid::assignOptimal(g, robots, orders);
        const auto expected = bestMatching(g, robots, orders);
        CHECK(static_cast<int>(a.size()) == expected.first);
        CHECK(totalDistance(a) == expected.second);
        std::vector<bool> robotUsed(robots.size(), false), orderUsed(orders.size(), false);
        for (const Assignment& x : a) {
            CHECK_FALSE(robotUsed[static_cast<std::size_t>(x.robot)]);
            CHECK_FALSE(orderUsed[static_cast<std::size_t>(x.order)]);
            robotUsed[static_cast<std::size_t>(x.robot)] = true;
            orderUsed[static_cast<std::size_t>(x.order)] = true;
            CHECK(x.distance == swarmgrid::bfsDistances(g, orders[static_cast<std::size_t>(x.order)])
                                    [static_cast<std::size_t>(robots[static_cast<std::size_t>(x.robot)])]);
        }
        // Greedy never beats it (fewer pairs, or as many pairs and a total at least as large).
        const auto greedy = swarmgrid::assignNearest(g, robots, orders);
        CHECK(greedy.size() <= a.size());
        if (greedy.size() == a.size()) CHECK(totalDistance(greedy) >= totalDistance(a));
    }
}

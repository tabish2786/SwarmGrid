#include "doctest/doctest.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <set>
#include <stdexcept>
#include <vector>

#include "swarmgrid/constraints.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/mdd.hpp"
#include "swarmgrid/space_time_astar.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::Constraint;
using swarmgrid::ConstraintTable;
using swarmgrid::Grid;
using swarmgrid::Mdd;

namespace {

std::vector<std::size_t> levelSizes(const Mdd& m) {
    std::vector<std::size_t> out;
    for (const auto& level : m.levels) out.push_back(level.size());
    return out;
}

// Brute force: walk every timed path of exactly `cost` steps and record the cells used at each
// time by the valid ones. Shares nothing with buildMdd except Grid and ConstraintTable.
std::vector<std::set<CellId>> bruteForceLevels(const Grid& g, CellId start, CellId goal, int cost,
                                               const ConstraintTable& ct) {
    std::vector<std::set<CellId>> levels(static_cast<std::size_t>(cost) + 1);
    if (ct.blockedFromAt(goal) >= 0 || cost <= ct.latestVertexConstraintAt(goal)) return levels;
    std::vector<CellId> path{start};
    std::vector<CellId> nb;
    std::function<void()> walk = [&] {
        const int t = static_cast<int>(path.size()) - 1;
        if (t == cost) {
            if (path.back() != goal) return;
            for (std::size_t i = 0; i < path.size(); ++i) levels[i].insert(path[i]);
            return;
        }
        std::vector<CellId> moves;
        g.neighbors(path.back(), nb);
        moves = nb;
        moves.push_back(path.back());
        for (CellId m : moves) {
            if (ct.vertexBlocked(m, t + 1)) continue;
            if (m != path.back() && ct.edgeBlocked(path.back(), m, t + 1)) continue;
            path.push_back(m);
            walk();
            path.pop_back();
        }
    };
    if (!ct.vertexBlocked(start, 0)) walk();
    return levels;
}

}  // namespace

TEST_CASE("mdd: open 3x3, corner to corner") {
    Grid g(3, 3);
    const CellId s = g.toId(0, 0), goal = g.toId(2, 2);
    const auto h = swarmgrid::bfsDistances(g, goal);
    const Mdd m = swarmgrid::buildMdd(g, s, goal, 4, h, ConstraintTable{});
    CHECK(levelSizes(m) == std::vector<std::size_t>{1, 2, 3, 2, 1});
    CHECK(m.levels[2] == std::vector<CellId>{g.toId(2, 0), g.toId(1, 1), g.toId(0, 2)});
    // One step slower than optimal: waits appear, the start is still possible at t=1.
    const Mdd slow = swarmgrid::buildMdd(g, s, goal, 5, h, ConstraintTable{});
    REQUIRE(slow.levels.size() == 6);
    CHECK(std::count(slow.levels[1].begin(), slow.levels[1].end(), s) == 1);
    // Faster than possible: no MDD.
    CHECK(swarmgrid::buildMdd(g, s, goal, 3, h, ConstraintTable{}).levels.empty());
}

TEST_CASE("mdd: a vertex constraint removes the centre") {
    Grid g(3, 3);
    const CellId s = g.toId(0, 0), goal = g.toId(2, 2);
    ConstraintTable ct;
    ct.add(swarmgrid::vertexConstraint(0, g.toId(1, 1), 2));
    const Mdd m = swarmgrid::buildMdd(g, s, goal, 4, swarmgrid::bfsDistances(g, goal), ct);
    CHECK(m.levels[2] == std::vector<CellId>{g.toId(2, 0), g.toId(0, 2)});
    CHECK(m.levels[1] == std::vector<CellId>{g.toId(1, 0), g.toId(0, 1)});
    CHECK(m.levels[3] == std::vector<CellId>{g.toId(2, 1), g.toId(1, 2)});
}

TEST_CASE("mdd: corridor is all singletons; start == goal has one level") {
    Grid g(5, 1);
    const auto h = swarmgrid::bfsDistances(g, 4);
    const Mdd m = swarmgrid::buildMdd(g, 0, 4, 4, h, ConstraintTable{});
    CHECK(levelSizes(m) == std::vector<std::size_t>{1, 1, 1, 1, 1});
    const Mdd stay = swarmgrid::buildMdd(g, 2, 2, 0, swarmgrid::bfsDistances(g, 2), ConstraintTable{});
    REQUIRE(stay.levels.size() == 1);
    CHECK(stay.levels[0] == std::vector<CellId>{2});
}

TEST_CASE("mdd: goal condition and edge constraints") {
    Grid g(3, 1);
    const auto h = swarmgrid::bfsDistances(g, 2);
    ConstraintTable onGoal;
    onGoal.add(swarmgrid::vertexConstraint(0, 2, 2));
    CHECK(swarmgrid::buildMdd(g, 0, 2, 2, h, onGoal).levels.empty());  // would stand on it at t=2
    CHECK(levelSizes(swarmgrid::buildMdd(g, 0, 2, 3, h, onGoal)).size() == 4);
    ConstraintTable forever;
    forever.add(swarmgrid::blockedFromConstraint(0, 2, 5));
    CHECK(swarmgrid::buildMdd(g, 0, 2, 9, h, forever).levels.empty());
    ConstraintTable edge;
    edge.add(swarmgrid::edgeConstraint(0, 0, 1, 1));
    CHECK(swarmgrid::buildMdd(g, 0, 2, 2, h, edge).levels.empty());
    const Mdd m = swarmgrid::buildMdd(g, 0, 2, 3, h, edge);
    REQUIRE(m.levels.size() == 4);
    CHECK(m.levels[0] == std::vector<CellId>{0});
    CHECK(m.levels[1] == std::vector<CellId>{0});  // must wait: 0->1 is forbidden at t=1
    CHECK(m.levels[2] == std::vector<CellId>{1});
    CHECK(m.levels[3] == std::vector<CellId>{2});
}

TEST_CASE("mdd: bad input throws") {
    Grid g(3, 1);
    g.setObstacle(1, 0, true);
    const auto h = swarmgrid::bfsDistances(g, 2);
    CHECK_THROWS_AS(swarmgrid::buildMdd(g, 1, 2, 2, h, ConstraintTable{}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::buildMdd(g, 0, 2, -1, h, ConstraintTable{}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::buildMdd(g, 0, 2, 2, std::vector<int>(2, 0), ConstraintTable{}),
                    std::invalid_argument);
    CHECK(swarmgrid::buildMdd(g, 0, 2, 5, h, ConstraintTable{}).levels.empty());  // unreachable
}

TEST_CASE("mdd: matches brute-force enumeration on random small grids") {
    swarmgrid::Rng rng(2024);
    int checked = 0;
    for (int trial = 0; trial < 150; ++trial) {
        Grid g(4, 3);
        for (int i = rng.uniformInt(0, 3); i > 0; --i) {
            const int x = rng.uniformInt(0, 3);  // separate statements: argument order is unspecified
            const int y = rng.uniformInt(0, 2);
            g.setObstacle(x, y, true);
        }
        const CellId s = rng.uniformInt(0, g.numCells() - 1);
        const CellId goal = rng.uniformInt(0, g.numCells() - 1);
        if (!g.isFree(s) || !g.isFree(goal)) continue;
        const auto h = swarmgrid::bfsDistances(g, goal);
        if (h[static_cast<std::size_t>(s)] == swarmgrid::kUnreachable) continue;
        ConstraintTable ct;
        for (int i = rng.uniformInt(0, 4); i > 0; --i) {
            const CellId c = rng.uniformInt(0, g.numCells() - 1);
            const int t = rng.uniformInt(1, 6);
            if (rng.uniformInt(0, 1) == 0) {
                ct.add(swarmgrid::vertexConstraint(0, c, t));
            } else {
                std::vector<CellId> nb;
                g.neighbors(c, nb);
                if (!nb.empty()) ct.add(swarmgrid::edgeConstraint(0, c, nb[0], t));
            }
        }
        // The cheapest valid cost (from space-time A*) and the two above it.
        const auto best = swarmgrid::spaceTimeAstar(g, s, goal, h, ct);
        if (!best) continue;
        const int optimal = static_cast<int>(best->size()) - 1;
        for (int cost = optimal; cost <= std::min(optimal + 2, 8); ++cost) {
            const auto expected = bruteForceLevels(g, s, goal, cost, ct);
            const Mdd m = swarmgrid::buildMdd(g, s, goal, cost, h, ct);
            if (expected.front().empty()) {
                CHECK(m.levels.empty());
                continue;
            }
            REQUIRE(m.levels.size() == expected.size());
            for (std::size_t t = 0; t < expected.size(); ++t) {
                CHECK(std::vector<CellId>(expected[t].begin(), expected[t].end()) == m.levels[t]);
            }
            ++checked;
        }
        CHECK(!swarmgrid::buildMdd(g, s, goal, optimal, h, ct).levels.empty());
    }
    CHECK(checked > 100);
}

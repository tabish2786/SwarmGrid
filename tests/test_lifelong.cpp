#include "doctest/doctest.h"

#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "swarmgrid/lifelong.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid::LifelongOptions;
using swarmgrid::LifelongSolver;
using swarmgrid::LifelongStats;

namespace {

// Re-checks the recorded motion without trusting the simulator: every robot moves at most one
// cell per step onto a free cell, no two robots share a cell, and no two swap.
void requireSafeMotion(const Grid& g, const LifelongStats& s, int robots) {
    REQUIRE(static_cast<int>(s.positions.size()) == s.steps + 1);
    int bad = 0;  // counted, then checked once: millions of tiny CHECKs would drown the output
    for (std::size_t t = 0; t < s.positions.size(); ++t) {
        const auto& now = s.positions[t];
        REQUIRE(static_cast<int>(now.size()) == robots);
        for (std::size_t i = 0; i < now.size(); ++i) {
            if (!g.isFree(now[i])) ++bad;
            for (std::size_t j = i + 1; j < now.size(); ++j) bad += now[i] == now[j] ? 1 : 0;
            if (t == 0) continue;
            const auto& before = s.positions[t - 1];
            const swarmgrid::Point a = g.toPoint(before[i]), b = g.toPoint(now[i]);
            if (std::abs(a.x - b.x) + std::abs(a.y - b.y) > 1) ++bad;
            for (std::size_t j = 0; j < now.size(); ++j) {
                if (j != i && now[i] != before[i] && now[i] == before[j] && now[j] == before[i]) ++bad;
            }
        }
    }
    CHECK(bad == 0);
}

LifelongOptions small(LifelongSolver solver) {
    LifelongOptions o;
    o.robots = 10;
    o.steps = 200;
    o.window = 8;
    o.replanEvery = 4;
    o.solver = solver;
    return o;
}

}  // namespace

TEST_CASE("lifelong: every solver delivers orders without collisions") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    for (LifelongSolver solver : {LifelongSolver::Prioritized, LifelongSolver::Cbs, LifelongSolver::Ecbs}) {
        CAPTURE(swarmgrid::solverName(solver));
        const LifelongStats s = swarmgrid::runLifelong(g, small(solver), true);
        CHECK(s.collisions == 0);
        CHECK(s.delivered > 10);
        CHECK(s.replans >= 200 / 4);
        CHECK(s.allWait == 0);
        requireSafeMotion(g, s, 10);
    }
}

TEST_CASE("lifelong: deterministic for a fixed seed") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const LifelongStats a = swarmgrid::runLifelong(g, small(LifelongSolver::Ecbs), true);
    const LifelongStats b = swarmgrid::runLifelong(g, small(LifelongSolver::Ecbs), true);
    CHECK(a.delivered == b.delivered);
    CHECK(a.positions == b.positions);
    LifelongOptions other = small(LifelongSolver::Ecbs);
    other.seed = 2;
    CHECK(swarmgrid::runLifelong(g, other, true).positions != a.positions);
}

TEST_CASE("lifelong: breakdowns, full-horizon planning and a crowded floor stay collision-free") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    SUBCASE("breakdowns") {
        LifelongOptions o = small(LifelongSolver::Ecbs);
        o.breakdownRate = 0.005;
        o.repairSteps = 15;
        const LifelongStats s = swarmgrid::runLifelong(g, o, true);
        CHECK(s.breakdowns > 0);
        CHECK(s.collisions == 0);
        CHECK(s.delivered > 0);
        requireSafeMotion(g, s, 10);
    }
    SUBCASE("window = -1 (whole paths)") {
        LifelongOptions o = small(LifelongSolver::Prioritized);
        o.window = -1;
        const LifelongStats s = swarmgrid::runLifelong(g, o, true);
        CHECK(s.collisions == 0);
        CHECK(s.delivered > 0);
        requireSafeMotion(g, s, 10);
    }
    SUBCASE("40 robots") {
        LifelongOptions o = small(LifelongSolver::Ecbs);
        o.robots = 40;
        o.steps = 150;
        const LifelongStats s = swarmgrid::runLifelong(g, o, true);
        CHECK(s.collisions == 0);
        CHECK(s.delivered > 0);
        requireSafeMotion(g, s, 40);
    }
}

TEST_CASE("lifelong: invalid options throw") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    LifelongOptions o = small(LifelongSolver::Ecbs);
    o.replanEvery = 10;  // > window
    CHECK_THROWS_AS(swarmgrid::runLifelong(g, o), std::invalid_argument);
    o = small(LifelongSolver::Ecbs);
    o.robots = 0;
    CHECK_THROWS_AS(swarmgrid::runLifelong(g, o), std::invalid_argument);
    o = small(LifelongSolver::Ecbs);
    o.breakdownRate = 2;
    CHECK_THROWS_AS(swarmgrid::runLifelong(g, o), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::runLifelong(Grid(3, 3), small(LifelongSolver::Ecbs)), std::invalid_argument);
}

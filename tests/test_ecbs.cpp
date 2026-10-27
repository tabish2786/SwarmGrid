#include "doctest/doctest.h"

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/space_time_astar.hpp"
#include "swarmgrid/warehouse_gen.hpp"
#include "mapf_reference.hpp"

using swarmgrid::CellId;
using swarmgrid::ConflictAvoidanceTable;
using swarmgrid::ConstraintTable;
using swarmgrid::EcbsOptions;
using swarmgrid::EcbsResult;
using swarmgrid::Grid;
using swarmgrid::Path;
using swarmgrid_test::referenceSumOfCosts;

namespace {

void instanceIds(const Grid& g, const swarmgrid::MapfInstance& inst, std::vector<CellId>& s,
                 std::vector<CellId>& t) {
    s.clear();
    t.clear();
    for (std::size_t i = 0; i < inst.starts.size(); ++i) {
        s.push_back(g.toId(inst.starts[i]));
        t.push_back(g.toId(inst.goals[i]));
    }
}

EcbsOptions withWeight(double w) {
    EcbsOptions o;
    o.weight = w;
    return o;
}

}  // namespace

TEST_CASE("cat: counts robots on a cell, parked robots and swaps") {
    Grid g(4, 1);
    ConflictAvoidanceTable cat;
    cat.addPath(Path{0, 1, 2});  // at 0 (t0), 1 (t1), parks on 2 from t2
    CHECK(cat.stepConflicts(1, 0, 0) == 1);   // on 0 at t0
    CHECK(cat.stepConflicts(0, 1, 1) == 1);   // on 1 at t1
    CHECK(cat.stepConflicts(3, 2, 1) == 0);   // 2 is free at t1
    CHECK(cat.stepConflicts(3, 2, 2) == 1);   // parked from t2 ...
    CHECK(cat.stepConflicts(3, 2, 50) == 1);  // ... forever
    CHECK(cat.stepConflicts(2, 1, 2) == 1);   // swap: it moves 1 -> 2 at t2 while we move 2 -> 1
    CHECK(cat.stepConflicts(1, 1, 2) == 0);   // waiting on 1 at t2: it has left
    CHECK(cat.visitsAfter(1, 0) == 1);
    CHECK(cat.visitsAfter(1, 1) == 0);
    CHECK(cat.visitsAfter(3, 0) == 0);
}

TEST_CASE("focal search: weight 1 is optimal; a larger weight trades length for fewer conflicts") {
    //   . . . . .
    //   . @ @ @ .
    //   . . . . .
    Grid g(5, 3);
    for (int x = 1; x <= 3; ++x) g.setObstacle(x, 1, true);
    const CellId s = g.toId(0, 1), goal = g.toId(4, 1);
    const auto h = swarmgrid::bfsDistances(g, goal);
    // Another robot drives along the top row and parks at its right end; the bottom row is free.
    ConflictAvoidanceTable cat;
    cat.addPath(Path{g.toId(0, 0), g.toId(1, 0), g.toId(2, 0), g.toId(3, 0), g.toId(4, 0)});
    const auto opt = swarmgrid::spaceTimeAstar(g, s, goal, h, ConstraintTable{});
    REQUIRE(opt);
    const auto w1 = swarmgrid::focalSpaceTimeSearch(g, s, goal, h, ConstraintTable{}, cat, 1.0);
    REQUIRE(w1);
    CHECK(w1->path.size() == opt->size());
    CHECK(w1->lowerBound == 6);
    CHECK(w1->conflicts == 0);  // both rows are length 6; it takes the bottom one
    CHECK(w1->path[2] == g.toId(1, 2));

    CHECK_THROWS_AS(swarmgrid::focalSpaceTimeSearch(g, s, goal, h, ConstraintTable{}, cat, 0.9),
                    std::invalid_argument);
}

TEST_CASE("focal search: a larger weight waits to avoid a conflict that weight 1 must accept") {
    //   . . . . .    we drive (0,0) -> (4,0): 4 steps, on (2,0) at t = 2
    //   @ @ . @ @    another robot pops out of the pocket onto (2,0) at t = 1..2, then goes back
    Grid g(5, 2);
    for (int x : {0, 1, 3, 4}) g.setObstacle(x, 1, true);
    const CellId s = g.toId(0, 0), goal = g.toId(4, 0);
    const auto h = swarmgrid::bfsDistances(g, goal);
    ConflictAvoidanceTable cat;
    cat.addPath(Path{g.toId(2, 1), g.toId(2, 0), g.toId(2, 0), g.toId(2, 1)});
    const auto strict = swarmgrid::focalSpaceTimeSearch(g, s, goal, h, ConstraintTable{}, cat, 1.0);
    const auto loose = swarmgrid::focalSpaceTimeSearch(g, s, goal, h, ConstraintTable{}, cat, 2.0);
    REQUIRE(strict);
    REQUIRE(loose);
    CHECK(strict->path.size() == 5);  // every 4-step path is on (2,0) at t = 2
    CHECK(strict->conflicts == 1);
    CHECK(loose->conflicts == 0);     // waits at the start, then drives through
    CHECK(loose->lowerBound == 4);
    CHECK(loose->path.size() > 5);
    CHECK(static_cast<int>(loose->path.size()) - 1 <= 2 * loose->lowerBound);
}

TEST_CASE("focal search: same validity rules as space-time A*") {
    Grid g(3, 1);
    const auto h = swarmgrid::bfsDistances(g, 2);
    ConstraintTable onGoal;
    onGoal.add(swarmgrid::vertexConstraint(0, 2, 5));
    const auto r = swarmgrid::focalSpaceTimeSearch(g, 0, 2, h, onGoal, ConflictAvoidanceTable{}, 1.5);
    REQUIRE(r);
    CHECK(r->path.size() == 7);  // must arrive after t = 5
    CHECK(r->lowerBound == 6);
    ConstraintTable forever;
    forever.add(swarmgrid::blockedFromConstraint(0, 2, 3));
    CHECK_FALSE(swarmgrid::focalSpaceTimeSearch(g, 0, 2, h, forever, ConflictAvoidanceTable{}, 1.5));
}

TEST_CASE("ecbs: invalid input throws") {
    Grid g(3, 1);
    CHECK_THROWS_AS(swarmgrid::solveEcbs(g, {0, 0}, {1, 2}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::solveEcbs(g, {0}, {2}, withWeight(0.5)), std::invalid_argument);
    EcbsOptions bad;
    bad.maxNodes = 0;
    CHECK_THROWS_AS(swarmgrid::solveEcbs(g, {0}, {2}, bad), std::invalid_argument);
}

TEST_CASE("ecbs: weight 1 is optimal, larger weights stay within the bound (random small instances)") {
    swarmgrid::Rng rng(4242);
    int checked = 0;
    for (int trial = 0; trial < 60; ++trial) {
        Grid g(4, 4);
        for (int i = rng.uniformInt(0, 3); i > 0; --i) {
            const int x = rng.uniformInt(0, 3);
            const int y = rng.uniformInt(0, 3);
            g.setObstacle(x, y, true);
        }
        const int k = rng.uniformInt(2, 3);
        swarmgrid::MapfInstance inst;
        try {
            inst = swarmgrid::generateRandomInstance(g, k, static_cast<std::uint32_t>(trial + 100));
        } catch (const std::invalid_argument&) {
            continue;
        }
        std::vector<CellId> s, t;
        instanceIds(g, inst, s, t);
        const long long opt = referenceSumOfCosts(g, s, t);
        for (double w : {1.0, 1.1, 1.5}) {
            EcbsOptions o = withWeight(w);
            o.maxNodes = 5000;
            const EcbsResult r = swarmgrid::solveEcbs(g, s, t, o);
            if (opt < 0) {
                CHECK_FALSE(r.solved);
                continue;
            }
            REQUIRE(r.solved);
            const auto report = swarmgrid::validatePlan(g, s, t, r.paths);
            CHECK(report.ok);
            CHECK(report.sumOfCosts == r.sumOfCosts);
            CHECK(r.lowerBound <= opt);
            CHECK(static_cast<double>(r.sumOfCosts) <= w * static_cast<double>(r.lowerBound) + 1e-9);
            CHECK(r.sumOfCosts >= opt);
            if (w == 1.0) CHECK(r.sumOfCosts == opt);
        }
        if (opt >= 0) ++checked;
    }
    CHECK(checked > 40);
}

TEST_CASE("ecbs: special maps (corridor pocket, goal blocking, plus intersection)") {
    {
        Grid g(5, 2);
        g.setObstacle(0, 1, true);
        for (int x = 2; x < 5; ++x) g.setObstacle(x, 1, true);
        const std::vector<CellId> s{g.toId(0, 0), g.toId(4, 0)}, t{g.toId(4, 0), g.toId(0, 0)};
        const EcbsResult r = swarmgrid::solveEcbs(g, s, t, withWeight(1.0));
        REQUIRE(r.solved);
        CHECK(r.sumOfCosts == 11);
    }
    {
        Grid g(5, 5);
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 5; ++x) {
                if (x != 2 && y != 2) g.setObstacle(x, y, true);
            }
        }
        const std::vector<CellId> s{g.toId(2, 0), g.toId(2, 4), g.toId(0, 2), g.toId(4, 2)};
        const std::vector<CellId> t{g.toId(2, 4), g.toId(2, 0), g.toId(4, 2), g.toId(0, 2)};
        for (double w : {1.0, 1.2}) {
            const EcbsResult r = swarmgrid::solveEcbs(g, s, t, withWeight(w));
            REQUIRE(r.solved);
            CHECK(swarmgrid::validatePlan(g, s, t, r.paths).ok);
            CHECK(r.sumOfCosts >= 28);
            CHECK(static_cast<double>(r.sumOfCosts) <= w * 28 + 1e-9);
            if (w == 1.0) CHECK(r.sumOfCosts == 28);
        }
    }
    {
        Grid g(2, 1);  // impossible swap: stops at the node limit
        EcbsOptions o;
        o.maxNodes = 300;
        const EcbsResult r = swarmgrid::solveEcbs(g, {0, 1}, {1, 0}, o);
        CHECK_FALSE(r.solved);
        CHECK(r.limitReached);
    }
}

TEST_CASE("ecbs: warehouse, 10 robots (seed 42) against CBS and prioritized") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    std::vector<CellId> s, t;
    instanceIds(g, swarmgrid::generateRandomInstance(g, 10, 42), s, t);
    const auto cbs = swarmgrid::solveCbs(g, s, t);
    REQUIRE(cbs.solved);
    CHECK(cbs.sumOfCosts == 208);
    for (double w : {1.0, 1.05, 1.2}) {
        const EcbsResult r = swarmgrid::solveEcbs(g, s, t, withWeight(w));
        REQUIRE(r.solved);
        CHECK(swarmgrid::validatePlan(g, s, t, r.paths).ok);
        CHECK(r.lowerBound <= cbs.sumOfCosts);
        CHECK(r.sumOfCosts >= cbs.sumOfCosts);
        CHECK(static_cast<double>(r.sumOfCosts) <= w * static_cast<double>(r.lowerBound) + 1e-9);
        if (w == 1.0) CHECK(r.sumOfCosts == 208);
    }
}

TEST_CASE("ecbs: 50 robots on the generated warehouse, w = 1.2") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    std::vector<CellId> s, t;
    instanceIds(g, swarmgrid::generateRandomInstance(g, 50, 7), s, t);
    EcbsOptions o = withWeight(1.2);
    o.maxNodes = 20000;
    const EcbsResult r = swarmgrid::solveEcbs(g, s, t, o);
    REQUIRE(r.solved);
    const auto report = swarmgrid::validatePlan(g, s, t, r.paths);
    CHECK(report.ok);
    CHECK(static_cast<double>(r.sumOfCosts) <= 1.2 * static_cast<double>(r.lowerBound) + 1e-9);
}

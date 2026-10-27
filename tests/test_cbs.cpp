#include "doctest/doctest.h"

#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/warehouse_gen.hpp"
#include "mapf_reference.hpp"

using swarmgrid::CbsOptions;
using swarmgrid::CbsResult;
using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid_test::referenceSumOfCosts;

namespace {

std::vector<CellId> ids(const Grid& g, std::vector<swarmgrid::Point> pts) {
    std::vector<CellId> out;
    for (auto p : pts) out.push_back(g.toId(p));
    return out;
}

// M4 plain CBS, each M5 improvement alone, and both (the default). All four must be optimal.
std::vector<CbsOptions> allVariants() {
    std::vector<CbsOptions> out;
    for (int mask = 0; mask < 4; ++mask) {
        CbsOptions o;
        o.prioritizeConflicts = (mask & 1) != 0;
        o.bypass = (mask & 2) != 0;
        out.push_back(o);
    }
    return out;
}

CbsOptions plainCbs() {
    CbsOptions o;
    o.prioritizeConflicts = false;
    o.bypass = false;
    return o;
}

void requireOptimalAndValid(const Grid& g, const std::vector<CellId>& s, const std::vector<CellId>& t,
                            const CbsResult& r, long long expected) {
    REQUIRE(r.solved);
    const auto report = swarmgrid::validatePlan(g, s, t, r.paths);
    CHECK(report.ok);
    CHECK(report.sumOfCosts == r.sumOfCosts);
    CHECK(r.sumOfCosts == expected);
}

}  // namespace

TEST_CASE("cbs: invalid input throws") {
    Grid g(3, 1);
    CHECK_THROWS_AS(swarmgrid::solveCbs(g, {0, 0}, {1, 2}), std::invalid_argument);
    CbsOptions bad;
    bad.maxNodes = 0;
    CHECK_THROWS_AS(swarmgrid::solveCbs(g, {0}, {2}, bad), std::invalid_argument);
}

TEST_CASE("cbs: corridor swap with one pocket") {
    //   . . . . .
    //   @ . @ @ @
    Grid g(5, 2);
    g.setObstacle(0, 1, true);
    for (int x = 2; x < 5; ++x) g.setObstacle(x, 1, true);
    const auto s = ids(g, {{0, 0}, {4, 0}});
    const auto t = ids(g, {{4, 0}, {0, 0}});
    for (const CbsOptions& o : allVariants()) {
        const CbsResult r = swarmgrid::solveCbs(g, s, t, o);
        requireOptimalAndValid(g, s, t, r, referenceSumOfCosts(g, s, t));
        CHECK(r.sumOfCosts == 11);  // one robot ducks into the pocket: 4 + 7
    }
}

TEST_CASE("cbs: goal-blocking (target conflict) — the parked robot must step aside") {
    //   . . . . .     robot 0: (1,0) -> (2,0), its goal is in robot 1's way
    //   @ @ . @ @     robot 1: (0,0) -> (4,0); (2,1) is a pocket
    Grid g(5, 2);
    for (int x : {0, 1, 3, 4}) g.setObstacle(x, 1, true);
    const auto s = ids(g, {{1, 0}, {0, 0}});
    const auto t = ids(g, {{2, 0}, {4, 0}});
    for (const CbsOptions& o : allVariants()) {
        const CbsResult r = swarmgrid::solveCbs(g, s, t, o);
        requireOptimalAndValid(g, s, t, r, referenceSumOfCosts(g, s, t));
        CHECK(r.sumOfCosts == 7);  // robot 0: (1,0) (2,0) pocket (2,0) = 3; robot 1: 4
    }
}

TEST_CASE("cbs: four robots crossing at a plus-shaped intersection") {
    //   @ @ . @ @
    //   @ @ . @ @
    //   . . . . .
    //   @ @ . @ @
    //   @ @ . @ @
    Grid g(5, 5);
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            if (x != 2 && y != 2) g.setObstacle(x, y, true);
        }
    }
    const auto s = ids(g, {{2, 0}, {2, 4}, {0, 2}, {4, 2}});
    const auto t = ids(g, {{2, 4}, {2, 0}, {4, 2}, {0, 2}});
    const long long expected = referenceSumOfCosts(g, s, t);
    CHECK(expected == 28);  // 16 if they could drive straight through each other
    for (const CbsOptions& o : allVariants()) {
        requireOptimalAndValid(g, s, t, swarmgrid::solveCbs(g, s, t, o), expected);
    }
    const CbsResult plain = swarmgrid::solveCbs(g, s, t, plainCbs());
    const CbsResult m5 = swarmgrid::solveCbs(g, s, t);
    CHECK(plain.nodesExpanded == 6474);  // M4
    CHECK(m5.nodesExpanded == 3784);     // M5: MDD conflict prioritization + bypass
}

TEST_CASE("cbs: unsolvable instance stops at the node limit") {
    //   . .    two robots that must swap with no room to pass
    Grid g(2, 1);
    CbsOptions opts;
    opts.maxNodes = 500;
    const CbsResult r = swarmgrid::solveCbs(g, {0, 1}, {1, 0}, opts);
    CHECK_FALSE(r.solved);
    CHECK(r.limitReached);
    CHECK(r.nodesExpanded == 500);
    CHECK(referenceSumOfCosts(g, {0, 1}, {1, 0}) == -1);
}

TEST_CASE("cbs: unreachable goal is reported at the root") {
    Grid g(3, 1);
    g.setObstacle(1, 0, true);
    const CbsResult r = swarmgrid::solveCbs(g, {0}, {2});
    CHECK_FALSE(r.solved);
    CHECK_FALSE(r.limitReached);
    CHECK(r.nodesExpanded == 0);
}

TEST_CASE("cbs: optimal against the joint-space reference on random small instances") {
    swarmgrid::Rng rng(31337);
    int checked = 0;
    for (int trial = 0; trial < 60; ++trial) {
        Grid g(4, 4);
        const int walls = rng.uniformInt(0, 3);
        for (int i = 0; i < walls; ++i) {
            // Two statements: the order of evaluation of function arguments is unspecified
            // (GCC and Clang differ), which would give each compiler different test grids.
            const int x = rng.uniformInt(0, 3);
            const int y = rng.uniformInt(0, 3);
            g.setObstacle(x, y, true);
        }
        const int k = rng.uniformInt(2, 3);
        swarmgrid::MapfInstance inst;
        try {
            inst = swarmgrid::generateRandomInstance(g, k, static_cast<std::uint32_t>(trial + 1));
        } catch (const std::invalid_argument&) {
            continue;  // too few free cells
        }
        std::vector<CellId> s, t;
        for (int i = 0; i < k; ++i) {
            s.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
            t.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
        }
        const long long expected = referenceSumOfCosts(g, s, t);
        for (CbsOptions opts : allVariants()) {
            opts.maxNodes = 5000;
            const CbsResult r = swarmgrid::solveCbs(g, s, t, opts);
            if (expected < 0) {
                CHECK_FALSE(r.solved);
                continue;
            }
            requireOptimalAndValid(g, s, t, r, expected);
        }
        if (expected >= 0) ++checked;
    }
    CHECK(checked > 40);
}

TEST_CASE("cbs: warehouse, 10 robots (seed 42): optimal, collision-free, golden numbers") {
    // The web dashboard runs this same code compiled to WebAssembly and must show these numbers.
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 10, 42);
    std::vector<CellId> s, t;
    for (int i = 0; i < 10; ++i) {
        s.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
        t.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
    }
    const CbsResult r = swarmgrid::solveCbs(g, s, t, plainCbs());
    REQUIRE(r.solved);
    CHECK(swarmgrid::validatePlan(g, s, t, r.paths).ok);
    const auto ind = swarmgrid::planMultiAgent(g, s, t, swarmgrid::MultiAgentMode::Independent);
    const auto pri = swarmgrid::planMultiAgent(g, s, t, swarmgrid::MultiAgentMode::Prioritized);
    long long indCost = 0, priCost = 0;
    for (const auto& p : ind.paths) indCost += static_cast<long long>(p.size()) - 1;
    for (const auto& p : pri.paths) priCost += static_cast<long long>(p.size()) - 1;
    CHECK(r.sumOfCosts >= indCost);  // nobody beats its own shortest path
    CHECK(r.sumOfCosts <= priCost);  // optimal can't be worse than prioritized
    CHECK(indCost == 200);
    CHECK(priCost == 212);
    CHECK(r.sumOfCosts == 208);
    CHECK(r.nodesExpanded == 494);  // M4 plain CBS
    CHECK(r.nodesGenerated == 987);

    // M5 finds a plan with the same (optimal) cost in fewer CT nodes.
    const CbsResult m5 = swarmgrid::solveCbs(g, s, t);
    REQUIRE(m5.solved);
    CHECK(swarmgrid::validatePlan(g, s, t, m5.paths).ok);
    CHECK(m5.sumOfCosts == 208);
    CHECK(m5.nodesExpanded == 245);
    CHECK(m5.nodesGenerated == 477);
    CHECK(m5.bypasses > 0);
    CHECK(m5.cardinalSplits > 0);
}

TEST_CASE("cbs: M5 counters add up") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 10, 42);
    std::vector<CellId> s, t;
    for (int i = 0; i < 10; ++i) {
        s.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
        t.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
    }
    const CbsResult m5 = swarmgrid::solveCbs(g, s, t);
    // Every expansion except the last (the solution) picks one conflict; some then bypass.
    CHECK(m5.cardinalSplits + m5.semiCardinalSplits + m5.nonCardinalSplits == m5.nodesExpanded - 1);
    CHECK(m5.bypasses < m5.nodesExpanded);
    const CbsResult plain = swarmgrid::solveCbs(g, s, t, plainCbs());
    CHECK(plain.mddsBuilt == 0);
    CHECK(plain.bypasses == 0);
    CHECK(plain.nonCardinalSplits == plain.nodesExpanded - 1);
}

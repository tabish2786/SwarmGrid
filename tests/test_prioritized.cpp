#include "doctest/doctest.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::Grid;
using swarmgrid::MultiAgentMode;
using swarmgrid::MultiAgentResult;
using swarmgrid::Path;

namespace {

// Written from scratch for the tests (M3 will add the real validator). A robot stays on the
// last cell of its path forever.
struct Conflicts {
    int vertex = 0;  // two robots on one cell at the same time (includes hitting a parked robot)
    int swap = 0;    // two robots exchange cells in one step
};

Conflicts countConflicts(const std::vector<Path>& paths) {
    std::size_t horizon = 0;
    for (const Path& p : paths) horizon = std::max(horizon, p.size());
    auto at = [&](std::size_t i, std::size_t t) {
        return t < paths[i].size() ? paths[i][t] : paths[i].back();
    };
    Conflicts c;
    for (std::size_t t = 0; t < horizon; ++t) {
        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (std::size_t j = i + 1; j < paths.size(); ++j) {
                if (at(i, t) == at(j, t)) ++c.vertex;
                if (t > 0 && at(i, t) != at(i, t - 1) && at(i, t) == at(j, t - 1) &&
                    at(j, t) == at(i, t - 1)) {
                    ++c.swap;
                }
            }
        }
    }
    return c;
}

// Right start and goal, every step a move to a free 4-neighbour or a wait.
bool validPaths(const Grid& g, const std::vector<Path>& paths, const std::vector<CellId>& starts,
                const std::vector<CellId>& goals) {
    if (paths.size() != starts.size()) return false;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const Path& p = paths[i];
        if (p.empty() || p.front() != starts[i] || p.back() != goals[i]) return false;
        for (std::size_t t = 0; t < p.size(); ++t) {
            if (!g.isFree(p[t])) return false;
            if (t > 0) {
                const swarmgrid::Point a = g.toPoint(p[t - 1]);
                const swarmgrid::Point b = g.toPoint(p[t]);
                if (std::abs(a.x - b.x) + std::abs(a.y - b.y) > 1) return false;
            }
        }
    }
    return true;
}

long long sumOfCosts(const std::vector<Path>& paths) {
    long long s = 0;
    for (const Path& p : paths) s += static_cast<long long>(p.size()) - 1;
    return s;
}

//   . . . . .
//   @ . @ @ @      one pocket at (1,1)
Grid pocketMap() {
    Grid g(5, 2);
    g.setObstacle(0, 1, true);
    for (int x = 2; x < 5; ++x) g.setObstacle(x, 1, true);
    return g;
}

}  // namespace

TEST_CASE("multi-agent: invalid input throws") {
    const Grid g = pocketMap();
    const CellId a = g.toId(0, 0);
    const CellId b = g.toId(4, 0);
    const CellId c = g.toId(2, 0);
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {}, {}, MultiAgentMode::Prioritized),
                    std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {a}, {b, c}, MultiAgentMode::Prioritized),
                    std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {a, a}, {b, c}, MultiAgentMode::Prioritized),
                    std::invalid_argument);  // shared start
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {a, c}, {b, b}, MultiAgentMode::Independent),
                    std::invalid_argument);  // shared goal
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {a}, {g.toId(2, 1)}, MultiAgentMode::Independent),
                    std::invalid_argument);  // goal on a shelf
    CHECK_THROWS_AS(swarmgrid::planMultiAgent(g, {a}, {b}, MultiAgentMode::Prioritized, 0),
                    std::invalid_argument);
}

TEST_CASE("multi-agent: head-on in a corridor with one pocket") {
    const Grid g = pocketMap();
    const std::vector<CellId> starts{g.toId(0, 0), g.toId(4, 0)};
    const std::vector<CellId> goals{g.toId(4, 0), g.toId(0, 0)};

    SUBCASE("independent planning collides") {
        const MultiAgentResult r = swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Independent);
        REQUIRE(r.solved);
        CHECK(sumOfCosts(r.paths) == 8);
        CHECK(countConflicts(r.paths).vertex > 0);  // they meet at (2,0) at t=2
    }
    SUBCASE("prioritized planning: order 0,1 fails, order 1,0 works with robot 0 in the pocket") {
        const MultiAgentResult r = swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Prioritized);
        REQUIRE(r.solved);
        CHECK(r.attempts == 2);
        CHECK(r.order == std::vector<int>{1, 0});
        CHECK(validPaths(g, r.paths, starts, goals));
        const Conflicts c = countConflicts(r.paths);
        CHECK(c.vertex == 0);
        CHECK(c.swap == 0);
        CHECK(r.paths[1].size() == 5);  // robot 1 drives straight: 4 moves
        CHECK(r.paths[0].size() == 8);  // robot 0: in at t=2, out at t=4, arrives at t=7
        CHECK(std::find(r.paths[0].begin(), r.paths[0].end(), g.toId(1, 1)) != r.paths[0].end());
    }
    SUBCASE("with a single attempt the first order fails, and the result says who failed") {
        const MultiAgentResult r =
            swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Prioritized, 1);
        CHECK_FALSE(r.solved);
        CHECK(r.paths.empty());
        CHECK(r.failedAgent == 1);
        CHECK(r.attempts == 1);
    }
}

TEST_CASE("multi-agent: an agent that can never reach its goal stops the search at once") {
    //   . @ .
    Grid g(3, 1);
    g.setObstacle(1, 0, true);
    const MultiAgentResult r = swarmgrid::planMultiAgent(g, {g.toId(0, 0)}, {g.toId(2, 0)},
                                                         MultiAgentMode::Prioritized, 10);
    CHECK_FALSE(r.solved);
    CHECK(r.failedAgent == 0);
    CHECK(r.attempts == 1);
}

TEST_CASE("multi-agent: random warehouse instances are collision-free when solved") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    int solved = 0;
    int tried = 0;
    for (int n : {10, 20, 30}) {
        for (std::uint32_t seed = 1; seed <= 5; ++seed) {
            const auto inst = swarmgrid::generateRandomInstance(g, n, seed);
            std::vector<CellId> starts;
            std::vector<CellId> goals;
            for (int i = 0; i < n; ++i) {
                starts.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
                goals.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
            }
            ++tried;
            const MultiAgentResult r =
                swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Prioritized);
            const MultiAgentResult ind =
                swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Independent);
            REQUIRE(ind.solved);
            if (!r.solved) continue;
            ++solved;
            CHECK(validPaths(g, r.paths, starts, goals));
            const Conflicts c = countConflicts(r.paths);
            CHECK(c.vertex == 0);
            CHECK(c.swap == 0);
            // Nobody can beat its own shortest path.
            CHECK(sumOfCosts(r.paths) >= sumOfCosts(ind.paths));
        }
    }
    CHECK(solved >= tried - 1);  // prioritized planning is incomplete, but not that weak here
}

TEST_CASE("multi-agent: golden result for the dashboard's default scene (10 robots, seed 42)") {
    // The web dashboard runs this same code compiled to WebAssembly and must print these numbers.
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 10, 42);
    std::vector<CellId> starts;
    std::vector<CellId> goals;
    for (int i = 0; i < 10; ++i) {
        starts.push_back(g.toId(inst.starts[static_cast<std::size_t>(i)]));
        goals.push_back(g.toId(inst.goals[static_cast<std::size_t>(i)]));
    }
    const MultiAgentResult ind = swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Independent);
    REQUIRE(ind.solved);
    CHECK(sumOfCosts(ind.paths) == 200);  // same as the dashboard's export-demos data

    const MultiAgentResult r = swarmgrid::planMultiAgent(g, starts, goals, MultiAgentMode::Prioritized);
    REQUIRE(r.solved);
    CHECK(countConflicts(r.paths).vertex == 0);
    CHECK(countConflicts(r.paths).swap == 0);
    CHECK(sumOfCosts(r.paths) == 212);  // 12 timesteps of waiting/detours buy zero collisions
    CHECK(r.attempts == 1);
    CHECK(r.expanded == 235);
}

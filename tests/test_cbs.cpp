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

using swarmgrid::CbsOptions;
using swarmgrid::CbsResult;
using swarmgrid::CellId;
using swarmgrid::Grid;

namespace {

// Independent reference for small instances: Dijkstra over the JOINT state of all agents.
// State = every agent's cell + which agents have "finished" (parked on their goal for good).
// Each timestep costs 1 per unfinished agent, so the cheapest way to finish everyone is the
// optimal sum of costs. Shares no code with CBS or space-time A*. Returns -1 if unsolvable.
long long referenceSumOfCosts(const Grid& g, const std::vector<CellId>& starts,
                              const std::vector<CellId>& goals) {
    const int k = static_cast<int>(starts.size());
    const auto V = static_cast<std::uint64_t>(g.numCells());
    const unsigned allDone = (1u << k) - 1;
    auto encode = [&](const std::vector<CellId>& pos, unsigned done) {
        std::uint64_t key = 0;
        for (CellId c : pos) key = key * V + static_cast<std::uint64_t>(c);
        return (key << k) | done;
    };
    struct Item {
        long long cost;
        std::vector<CellId> pos;
        unsigned done;
        bool operator>(const Item& o) const { return cost > o.cost; }
    };
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    std::unordered_map<std::uint64_t, long long> best;
    open.push({0, starts, 0u});
    best[encode(starts, 0u)] = 0;
    std::vector<CellId> nb;
    while (!open.empty()) {
        Item cur = open.top();
        open.pop();
        if (best[encode(cur.pos, cur.done)] < cur.cost) continue;
        if (cur.done == allDone) return cur.cost;
        auto relax = [&](const std::vector<CellId>& pos, unsigned done, long long cost) {
            const std::uint64_t key = encode(pos, done);
            auto it = best.find(key);
            if (it == best.end() || cost < it->second) {
                best[key] = cost;
                open.push({cost, pos, done});
            }
        };
        // Free action: an agent standing on its goal declares itself finished.
        for (int i = 0; i < k; ++i) {
            if (!(cur.done >> i & 1u) && cur.pos[static_cast<std::size_t>(i)] == goals[static_cast<std::size_t>(i)]) {
                relax(cur.pos, cur.done | (1u << i), cur.cost);
            }
        }
        // One timestep: every unfinished agent waits or moves; finished agents stay.
        std::vector<std::vector<CellId>> options(static_cast<std::size_t>(k));
        int unfinished = 0;
        for (int i = 0; i < k; ++i) {
            const CellId c = cur.pos[static_cast<std::size_t>(i)];
            options[static_cast<std::size_t>(i)] = {c};
            if (!(cur.done >> i & 1u)) {
                ++unfinished;
                g.neighbors(c, nb);
                for (CellId m : nb) options[static_cast<std::size_t>(i)].push_back(m);
            }
        }
        if (unfinished == 0) continue;
        std::vector<CellId> next(static_cast<std::size_t>(k));
        std::function<void(int)> choose = [&](int i) {
            if (i == k) {
                for (int a = 0; a < k; ++a) {
                    for (int b = a + 1; b < k; ++b) {
                        const auto ua = static_cast<std::size_t>(a), ub = static_cast<std::size_t>(b);
                        if (next[ua] == next[ub]) return;  // vertex conflict
                        if (next[ua] == cur.pos[ub] && next[ub] == cur.pos[ua]) return;  // swap
                    }
                }
                relax(next, cur.done, cur.cost + unfinished);
                return;
            }
            for (CellId c : options[static_cast<std::size_t>(i)]) {
                next[static_cast<std::size_t>(i)] = c;
                choose(i + 1);
            }
        };
        choose(0);
    }
    return -1;
}

std::vector<CellId> ids(const Grid& g, std::vector<swarmgrid::Point> pts) {
    std::vector<CellId> out;
    for (auto p : pts) out.push_back(g.toId(p));
    return out;
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
    const CbsResult r = swarmgrid::solveCbs(g, s, t);
    requireOptimalAndValid(g, s, t, r, referenceSumOfCosts(g, s, t));
    CHECK(r.sumOfCosts == 11);  // one robot ducks into the pocket: 4 + 7
}

TEST_CASE("cbs: goal-blocking (target conflict) — the parked robot must step aside") {
    //   . . . . .     robot 0: (1,0) -> (2,0), its goal is in robot 1's way
    //   @ @ . @ @     robot 1: (0,0) -> (4,0); (2,1) is a pocket
    Grid g(5, 2);
    for (int x : {0, 1, 3, 4}) g.setObstacle(x, 1, true);
    const auto s = ids(g, {{1, 0}, {0, 0}});
    const auto t = ids(g, {{2, 0}, {4, 0}});
    const CbsResult r = swarmgrid::solveCbs(g, s, t);
    requireOptimalAndValid(g, s, t, r, referenceSumOfCosts(g, s, t));
    CHECK(r.sumOfCosts == 7);  // robot 0: (1,0) (2,0) pocket (2,0) = 3; robot 1: 4
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
    const CbsResult r = swarmgrid::solveCbs(g, s, t);
    requireOptimalAndValid(g, s, t, r, referenceSumOfCosts(g, s, t));
    CHECK(r.sumOfCosts == 28);  // 16 if they could drive straight through each other
    CHECK(r.nodesExpanded == 6474);
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
            g.setObstacle(rng.uniformInt(0, 3), rng.uniformInt(0, 3), true);
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
        CbsOptions opts;
        opts.maxNodes = 5000;
        const CbsResult r = swarmgrid::solveCbs(g, s, t, opts);
        if (expected < 0) {
            CHECK_FALSE(r.solved);
            continue;
        }
        requireOptimalAndValid(g, s, t, r, expected);
        ++checked;
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
    const CbsResult r = swarmgrid::solveCbs(g, s, t);
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
    CHECK(r.nodesExpanded == 494);
    CHECK(r.nodesGenerated == 987);
}

#include "doctest/doctest.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "swarmgrid/constraints.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/space_time_astar.hpp"
#include "swarmgrid/warehouse_gen.hpp"

using swarmgrid::CellId;
using swarmgrid::ConstraintTable;
using swarmgrid::edgeConstraint;
using swarmgrid::Grid;
using swarmgrid::Path;
using swarmgrid::SearchStats;
using swarmgrid::vertexConstraint;

namespace {

// Starts at s, ends at goal, each step is a move to a 4-neighbour or a wait, every cell is
// free, no constraint is broken, and the agent may stay at the goal forever afterwards.
bool isValidTimedPath(const Grid& g, const Path& p, CellId s, CellId goal,
                      const ConstraintTable& ct) {
    if (p.empty() || p.front() != s || p.back() != goal) return false;
    for (std::size_t t = 0; t < p.size(); ++t) {
        if (!g.isFree(p[t])) return false;
        if (ct.vertexBlocked(p[t], static_cast<int>(t))) return false;
        if (t > 0) {
            const swarmgrid::Point a = g.toPoint(p[t - 1]);
            const swarmgrid::Point b = g.toPoint(p[t]);
            const int d = std::abs(a.x - b.x) + std::abs(a.y - b.y);
            if (d > 1) return false;
            if (d == 1 && ct.edgeBlocked(p[t - 1], p[t], static_cast<int>(t))) return false;
        }
    }
    return static_cast<int>(p.size()) - 1 > ct.latestVertexConstraintAt(goal);
}

// Independent reference: sweep the set of reachable cells one timestep at a time. With unit
// costs, the first t at which the goal is reachable (and allowed to finish) is optimal.
// Returns the optimal arrival time, or -1 if there is none by the horizon.
int referenceArrival(const Grid& g, CellId s, CellId goal, const ConstraintTable& ct,
                     int horizon) {
    if (ct.vertexBlocked(s, 0)) return -1;
    const auto n = static_cast<std::size_t>(g.numCells());
    std::vector<char> cur(n, 0);
    std::vector<char> nxt(n, 0);
    cur[static_cast<std::size_t>(s)] = 1;
    std::vector<CellId> nb;
    for (int t = 0; t <= horizon; ++t) {
        if (cur[static_cast<std::size_t>(goal)] && t > ct.latestVertexConstraintAt(goal)) return t;
        std::fill(nxt.begin(), nxt.end(), 0);
        for (CellId c = 0; c < g.numCells(); ++c) {
            if (!cur[static_cast<std::size_t>(c)]) continue;
            g.neighbors(c, nb);
            nb.push_back(c);
            for (CellId m : nb) {
                if (ct.vertexBlocked(m, t + 1)) continue;
                if (m != c && ct.edgeBlocked(c, m, t + 1)) continue;
                nxt[static_cast<std::size_t>(m)] = 1;
            }
        }
        cur.swap(nxt);
    }
    return -1;
}

Path cells(const Grid& g, const std::vector<int>& xs) {  // 1-row helper: x coordinates -> ids
    Path p;
    for (int x : xs) p.push_back(g.toId(x, 0));
    return p;
}

}  // namespace

TEST_CASE("space-time: no constraints equals the static shortest path") {
    const Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const auto inst = swarmgrid::generateRandomInstance(g, 30, 11);
    const ConstraintTable none;
    for (int i = 0; i < inst.numAgents(); ++i) {
        const CellId s = g.toId(inst.starts[static_cast<std::size_t>(i)]);
        const CellId t = g.toId(inst.goals[static_cast<std::size_t>(i)]);
        const auto h = swarmgrid::bfsDistances(g, t);
        SearchStats stats;
        const auto path = swarmgrid::spaceTimeAstar(g, s, t, h, none, -1, &stats);
        REQUIRE(path.has_value());
        CHECK(static_cast<int>(path->size()) - 1 == h[s]);
        CHECK(isValidTimedPath(g, *path, s, t, none));
        CHECK(stats.expanded == static_cast<int>(path->size()));  // exact h: walks one path
    }
}

TEST_CASE("space-time: corridor examples (exact paths)") {
    const Grid g(5, 1);  // S . . . G
    const CellId s = g.toId(0, 0);
    const CellId goal = g.toId(4, 0);
    const auto h = swarmgrid::bfsDistances(g, goal);

    SUBCASE("vertex constraint on the way forces one wait") {
        const ConstraintTable ct({vertexConstraint(0, g.toId(2, 0), 2)}, 0);
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, h, ct);
        REQUIRE(path.has_value());
        CHECK(*path == cells(g, {0, 1, 1, 2, 3, 4}));
    }
    SUBCASE("edge constraint on the first move forces a wait at the start") {
        const ConstraintTable ct({edgeConstraint(0, g.toId(0, 0), g.toId(1, 0), 1)}, 0);
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, h, ct);
        REQUIRE(path.has_value());
        CHECK(*path == cells(g, {0, 0, 1, 2, 3, 4}));
    }
    SUBCASE("edge constraint in the other direction changes nothing") {
        const ConstraintTable ct({edgeConstraint(0, g.toId(1, 0), g.toId(0, 0), 1)}, 0);
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, h, ct);
        REQUIRE(path.has_value());
        CHECK(*path == cells(g, {0, 1, 2, 3, 4}));
    }
    SUBCASE("constraint for another agent is ignored") {
        const ConstraintTable ct({vertexConstraint(1, g.toId(2, 0), 2)}, 0);
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, h, ct);
        REQUIRE(path.has_value());
        CHECK(path->size() == 5);
    }
    SUBCASE("goal condition: cannot finish before a later constraint on the goal") {
        // Arriving at t=4 and staying would be standing on the goal at t=6.
        const ConstraintTable ct({vertexConstraint(0, goal, 6)}, 0);
        SearchStats stats;
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, h, ct, -1, &stats);
        REQUIRE(path.has_value());
        CHECK(path->size() == 8);  // arrival 7
        CHECK((*path)[6] != goal);
        CHECK(isValidTimedPath(g, *path, s, goal, ct));
    }
}

TEST_CASE("space-time: start == goal with a later constraint on it must step away") {
    const Grid g(3, 1);
    const CellId c = g.toId(1, 0);
    const auto h = swarmgrid::bfsDistances(g, c);

    const auto stay = swarmgrid::spaceTimeAstar(g, c, c, h, ConstraintTable{});
    REQUIRE(stay.has_value());
    CHECK(*stay == Path{c});

    const ConstraintTable ct({vertexConstraint(0, c, 3)}, 0);
    const auto path = swarmgrid::spaceTimeAstar(g, c, c, h, ct);
    REQUIRE(path.has_value());
    CHECK(path->size() == 5);  // arrival 4: off the cell at t=3, back at t=4
    CHECK((*path)[3] != c);
    CHECK(isValidTimedPath(g, *path, c, c, ct));
}

TEST_CASE("space-time: infeasible cases") {
    const Grid g(5, 1);
    const CellId s = g.toId(0, 0);
    const CellId goal = g.toId(4, 0);
    const auto h = swarmgrid::bfsDistances(g, goal);

    SUBCASE("vertex constraint on the start at t=0") {
        const ConstraintTable ct({vertexConstraint(0, s, 0)}, 0);
        CHECK_FALSE(swarmgrid::spaceTimeAstar(g, s, goal, h, ct).has_value());
    }
    SUBCASE("explicit horizon too short") {
        const ConstraintTable none;
        CHECK_FALSE(swarmgrid::spaceTimeAstar(g, s, goal, h, none, 3).has_value());
        CHECK(swarmgrid::spaceTimeAstar(g, s, goal, h, none, 4).has_value());
    }
    SUBCASE("horizon counts the waits forced by constraints") {
        const ConstraintTable ct({vertexConstraint(0, g.toId(2, 0), 2)}, 0);
        CHECK_FALSE(swarmgrid::spaceTimeAstar(g, s, goal, h, ct, 4).has_value());
        CHECK(swarmgrid::spaceTimeAstar(g, s, goal, h, ct, 5).has_value());
    }
    SUBCASE("statically unreachable goal") {
        Grid walled(3, 1);
        walled.setObstacle(1, 0, true);
        const CellId t = walled.toId(2, 0);
        SearchStats stats;
        CHECK_FALSE(swarmgrid::spaceTimeAstar(walled, walled.toId(0, 0), t,
                                              swarmgrid::bfsDistances(walled, t),
                                              ConstraintTable{}, -1, &stats)
                        .has_value());
        CHECK(stats.expanded == 0);
    }
}

TEST_CASE("space-time: a blocked corridor cell for several steps means waiting it out") {
    //   S . X . G   with X blocked at t = 2, 3, 4: the agent can't sidestep in a corridor,
    //   so it must wait until X opens at t = 5 and arrives at t = 7.
    const Grid g(5, 1);
    const CellId x = g.toId(2, 0);
    const ConstraintTable ct(
        {vertexConstraint(0, x, 2), vertexConstraint(0, x, 3), vertexConstraint(0, x, 4)}, 0);
    const CellId goal = g.toId(4, 0);
    const auto path =
        swarmgrid::spaceTimeAstar(g, g.toId(0, 0), goal, swarmgrid::bfsDistances(g, goal), ct);
    REQUIRE(path.has_value());
    CHECK(path->size() == 8);
    CHECK(isValidTimedPath(g, *path, g.toId(0, 0), goal, ct));
}

TEST_CASE("space-time: optimal vs an independent reference on random constrained grids") {
    // 300 random 6x6 grids, random start/goal, up to 12 random vertex/edge constraints.
    swarmgrid::Rng rng(2024);
    int solved = 0;
    int infeasible = 0;
    for (int trial = 0; trial < 300; ++trial) {
        Grid g(6, 6);
        for (CellId c = 0; c < g.numCells(); ++c) {
            if (rng.uniformInt(0, 99) < 20) g.setObstacle(g.toPoint(c).x, g.toPoint(c).y, true);
        }
        std::vector<CellId> freeCells;
        for (CellId c = 0; c < g.numCells(); ++c) {
            if (g.isFree(c)) freeCells.push_back(c);
        }
        if (freeCells.size() < 2) continue;
        const int last = static_cast<int>(freeCells.size()) - 1;
        const CellId s = freeCells[static_cast<std::size_t>(rng.uniformInt(0, last))];
        const CellId goal = freeCells[static_cast<std::size_t>(rng.uniformInt(0, last))];

        ConstraintTable ct;
        const int k = rng.uniformInt(0, 12);
        std::vector<CellId> nb;
        for (int i = 0; i < k; ++i) {
            const CellId c = freeCells[static_cast<std::size_t>(rng.uniformInt(0, last))];
            const int t = rng.uniformInt(1, 10);
            g.neighbors(c, nb);
            if (rng.uniformInt(0, 1) == 0 || nb.empty()) {
                ct.add(vertexConstraint(0, c, t));
            } else {
                const int j = rng.uniformInt(0, static_cast<int>(nb.size()) - 1);
                ct.add(edgeConstraint(0, c, nb[static_cast<std::size_t>(j)], t));
            }
        }

        const int horizon = ct.maxTime() + g.numCells();  // the automatic horizon
        const int expected = referenceArrival(g, s, goal, ct, horizon);
        const auto path = swarmgrid::spaceTimeAstar(g, s, goal, swarmgrid::bfsDistances(g, goal), ct);
        if (expected < 0) {
            CHECK_FALSE(path.has_value());
            ++infeasible;
        } else {
            REQUIRE(path.has_value());
            CHECK(static_cast<int>(path->size()) - 1 == expected);
            CHECK(isValidTimedPath(g, *path, s, goal, ct));
            ++solved;
        }
    }
    CHECK(solved > 200);    // the generator must actually exercise the solver...
    CHECK(infeasible > 0);  // ...and the "no path" branch
}

TEST_CASE("space-time: invalid inputs throw") {
    const Grid g(3, 1);
    const auto h = swarmgrid::bfsDistances(g, g.toId(2, 0));
    const ConstraintTable none;
    CHECK_THROWS_AS(swarmgrid::spaceTimeAstar(g, -1, g.toId(2, 0), h, none), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::spaceTimeAstar(g, 0, g.numCells(), h, none), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::spaceTimeAstar(g, 0, 2, std::vector<int>(1, 0), none),
                    std::invalid_argument);
    Grid blocked(3, 1);
    blocked.setObstacle(0, 0, true);
    CHECK_THROWS_AS(swarmgrid::spaceTimeAstar(blocked, 0, 2, h, none), std::invalid_argument);
}

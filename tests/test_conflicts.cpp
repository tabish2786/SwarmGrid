#include "doctest/doctest.h"

#include <stdexcept>
#include <vector>

#include "swarmgrid/conflicts.hpp"

using swarmgrid::CellId;
using swarmgrid::Conflict;
using swarmgrid::ConflictType;
using swarmgrid::Grid;
using swarmgrid::Path;

namespace {

// 1-row helper: x coordinates on a width-w strip are also the cell ids.
Path xs(std::vector<int> v) { return Path(v.begin(), v.end()); }

}  // namespace

TEST_CASE("conflicts: none") {
    const std::vector<Path> paths{xs({0, 1, 2}), xs({5, 4, 3})};
    CHECK_FALSE(swarmgrid::findFirstConflict(paths).has_value());
    CHECK(swarmgrid::findAllConflicts(paths).empty());
    // Following someone (moving into the cell it just left) is allowed.
    CHECK_FALSE(swarmgrid::findFirstConflict({xs({1, 2, 3}), xs({0, 1, 2})}).has_value());
}

TEST_CASE("conflicts: vertex") {
    const auto c = swarmgrid::findFirstConflict({xs({0, 1, 2, 3}), xs({4, 3, 2, 1})});
    REQUIRE(c.has_value());
    CHECK(c->type == ConflictType::Vertex);
    CHECK(c->a1 == 0);
    CHECK(c->a2 == 1);
    CHECK(c->cell == 2);
    CHECK(c->t == 2);
}

TEST_CASE("conflicts: edge (swap)") {
    const auto c = swarmgrid::findFirstConflict({xs({0, 1, 2}), xs({3, 2, 1})});
    REQUIRE(c.has_value());
    CHECK(c->type == ConflictType::Edge);
    CHECK(c->a1 == 0);
    CHECK(c->a2 == 1);
    CHECK(c->cell == 1);  // a1 moves 1 -> 2 while a2 moves 2 -> 1
    CHECK(c->to == 2);
    CHECK(c->t == 2);
}

TEST_CASE("conflicts: target (an agent parked on its goal is hit later)") {
    // Agent 0 reaches its goal 2 at t=2 and stays; agent 1 drives through 2 at t=4.
    const auto c = swarmgrid::findFirstConflict({xs({0, 1, 2}), xs({6, 5, 4, 3, 2, 1})});
    REQUIRE(c.has_value());
    CHECK(c->type == ConflictType::Target);
    CHECK(c->a1 == 0);  // the parked one
    CHECK(c->a2 == 1);
    CHECK(c->cell == 2);
    CHECK(c->t == 4);

    // Same thing with the agent indices the other way round.
    const auto d = swarmgrid::findFirstConflict({xs({6, 5, 4, 3, 2, 1}), xs({0, 1, 2})});
    REQUIRE(d.has_value());
    CHECK(d->type == ConflictType::Target);
    CHECK(d->a1 == 1);
    CHECK(d->a2 == 0);
}

TEST_CASE("conflicts: order is earliest time, then agent pair, vertex before edge") {
    // t=1: agents 1 and 2 meet on cell 7. t=2: agents 0 and 1 would also meet. Earliest wins.
    const std::vector<Path> paths{xs({0, 1, 2}), xs({8, 7, 2}), xs({6, 7, 8})};
    const auto first = swarmgrid::findFirstConflict(paths);
    REQUIRE(first.has_value());
    CHECK(first->t == 1);
    CHECK(first->a1 == 1);
    CHECK(first->a2 == 2);
    const std::vector<Conflict> all = swarmgrid::findAllConflicts(paths);
    REQUIRE(all.size() >= 2);
    CHECK(all[0].t == 1);
    CHECK(all[1].t == 2);
    for (std::size_t i = 1; i < all.size(); ++i) CHECK(all[i - 1].t <= all[i].t);

    // Three robots on one cell are three pairwise conflicts.
    CHECK(swarmgrid::findAllConflicts({xs({5}), xs({5}), xs({5})}).size() == 3);
}

TEST_CASE("validator: accepts a good plan, explains a bad one") {
    //   . . .
    //   . @ .
    Grid g(3, 2);
    g.setObstacle(1, 1, true);
    const std::vector<CellId> starts{g.toId(0, 0), g.toId(2, 0)};
    const std::vector<CellId> goals{g.toId(0, 1), g.toId(2, 1)};

    SUBCASE("good") {
        const auto r = swarmgrid::validatePlan(g, starts, goals, {{g.toId(0, 0), g.toId(0, 1)},
                                                                 {g.toId(2, 0), g.toId(2, 1)}});
        CHECK(r.ok);
        CHECK(r.errors.empty());
        CHECK(r.sumOfCosts == 2);
        CHECK(r.makespan == 1);
    }
    SUBCASE("wrong goal, jump, obstacle") {
        const auto wrongGoal = swarmgrid::validatePlan(g, starts, goals, {{g.toId(0, 0)}, {g.toId(2, 0), g.toId(2, 1)}});
        CHECK_FALSE(wrongGoal.ok);
        CHECK(wrongGoal.errors.size() == 1);
        const auto jump = swarmgrid::validatePlan(g, starts, goals, {{g.toId(0, 0), g.toId(0, 1)}, {g.toId(2, 0), g.toId(0, 0), g.toId(2, 1)}});
        CHECK_FALSE(jump.ok);
        const auto wall = swarmgrid::validatePlan(g, starts, goals, {{g.toId(0, 0), g.toId(0, 1)}, {g.toId(2, 0), g.toId(1, 1), g.toId(2, 1)}});
        CHECK_FALSE(wall.ok);
        const auto count = swarmgrid::validatePlan(g, starts, goals, {{g.toId(0, 0), g.toId(0, 1)}});
        CHECK_FALSE(count.ok);
    }
    SUBCASE("conflicts are reported") {
        // Both go through (1,0) at t=1.
        const std::vector<CellId> s{g.toId(0, 0), g.toId(2, 0)};
        const std::vector<CellId> t{g.toId(2, 1), g.toId(0, 1)};
        const auto r = swarmgrid::validatePlan(g, s, t,
            {{g.toId(0, 0), g.toId(1, 0), g.toId(2, 0), g.toId(2, 1)},
             {g.toId(2, 0), g.toId(1, 0), g.toId(0, 0), g.toId(0, 1)}});
        CHECK_FALSE(r.ok);
        CHECK(r.errors.empty());
        CHECK_FALSE(r.conflicts.empty());
    }
}

TEST_CASE("requireValidAgents") {
    Grid g(3, 1);
    g.setObstacle(1, 0, true);
    CHECK_NOTHROW(swarmgrid::requireValidAgents(g, {0}, {2}, "x"));
    CHECK_THROWS_AS(swarmgrid::requireValidAgents(g, {}, {}, "x"), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::requireValidAgents(g, {0, 2}, {2}, "x"), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::requireValidAgents(g, {1}, {2}, "x"), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::requireValidAgents(g, {0, 2}, {2, 2}, "x"), std::invalid_argument);
}

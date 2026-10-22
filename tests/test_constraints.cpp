#include "doctest/doctest.h"

#include <stdexcept>
#include <vector>

#include "swarmgrid/constraints.hpp"

using swarmgrid::Constraint;
using swarmgrid::ConstraintTable;
using swarmgrid::ConstraintType;
using swarmgrid::edgeConstraint;
using swarmgrid::vertexConstraint;

TEST_CASE("constraints: factories fill the fields and validate") {
    const Constraint v = vertexConstraint(3, 17, 5);
    CHECK(v.type == ConstraintType::Vertex);
    CHECK(v.agent == 3);
    CHECK(v.from == 17);
    CHECK(v.to == swarmgrid::kInvalidCell);
    CHECK(v.t == 5);

    const Constraint e = edgeConstraint(1, 4, 5, 2);
    CHECK(e.type == ConstraintType::Edge);
    CHECK(e.from == 4);
    CHECK(e.to == 5);
    CHECK(e.t == 2);

    CHECK_NOTHROW(vertexConstraint(0, 0, 0));
    CHECK_THROWS_AS(vertexConstraint(0, -1, 0), std::invalid_argument);
    CHECK_THROWS_AS(vertexConstraint(0, 0, -1), std::invalid_argument);
    CHECK_THROWS_AS(edgeConstraint(0, 1, 1, 3), std::invalid_argument);  // a wait is not an edge
    CHECK_THROWS_AS(edgeConstraint(0, 1, 2, 0), std::invalid_argument);  // no move arrives at t=0
    CHECK_THROWS_AS(edgeConstraint(0, -1, 2, 1), std::invalid_argument);
}

TEST_CASE("constraint table: empty") {
    const ConstraintTable table;
    CHECK(table.empty());
    CHECK(table.maxTime() == -1);
    CHECK(table.latestVertexConstraintAt(0) == -1);
    CHECK_FALSE(table.vertexBlocked(0, 0));
    CHECK_FALSE(table.edgeBlocked(0, 1, 1));
}

TEST_CASE("constraint table: keeps only the requested agent") {
    const std::vector<Constraint> all{vertexConstraint(0, 5, 2), vertexConstraint(1, 6, 3),
                                      edgeConstraint(1, 6, 7, 4), edgeConstraint(0, 8, 9, 1)};
    const ConstraintTable t0(all, 0);
    CHECK(t0.vertexBlocked(5, 2));
    CHECK_FALSE(t0.vertexBlocked(6, 3));
    CHECK(t0.edgeBlocked(8, 9, 1));
    CHECK_FALSE(t0.edgeBlocked(6, 7, 4));
    CHECK(t0.maxTime() == 2);

    const ConstraintTable t1(all, 1);
    CHECK(t1.vertexBlocked(6, 3));
    CHECK(t1.edgeBlocked(6, 7, 4));
    CHECK_FALSE(t1.vertexBlocked(5, 2));
    CHECK(t1.maxTime() == 4);
}

TEST_CASE("constraint table: exact time and direction") {
    ConstraintTable table;
    table.add(vertexConstraint(0, 10, 3));
    table.add(edgeConstraint(0, 4, 5, 2));

    CHECK(table.vertexBlocked(10, 3));
    CHECK_FALSE(table.vertexBlocked(10, 2));
    CHECK_FALSE(table.vertexBlocked(10, 4));
    CHECK_FALSE(table.vertexBlocked(11, 3));
    CHECK_FALSE(table.vertexBlocked(-1, 3));
    CHECK_FALSE(table.vertexBlocked(10, -1));

    CHECK(table.edgeBlocked(4, 5, 2));
    CHECK_FALSE(table.edgeBlocked(5, 4, 2));  // the opposite direction is a different move
    CHECK_FALSE(table.edgeBlocked(4, 5, 1));
    CHECK_FALSE(table.edgeBlocked(4, 5, 3));
}

TEST_CASE("constraint table: latest vertex constraint per cell and maxTime") {
    ConstraintTable table;
    table.add(vertexConstraint(0, 7, 4));
    table.add(vertexConstraint(0, 7, 9));
    table.add(vertexConstraint(0, 7, 2));
    table.add(edgeConstraint(0, 7, 8, 12));  // edges don't count for the goal condition
    CHECK(table.latestVertexConstraintAt(7) == 9);
    CHECK(table.latestVertexConstraintAt(8) == -1);
    CHECK(table.maxTime() == 12);
    CHECK_FALSE(table.empty());
}

TEST_CASE("constraint table: adding a hand-built invalid constraint throws") {
    ConstraintTable table;
    Constraint bad;
    bad.type = ConstraintType::Edge;
    bad.from = 3;
    bad.to = 3;
    bad.t = 2;
    CHECK_THROWS_AS(table.add(bad), std::invalid_argument);
    CHECK(table.empty());
}

TEST_CASE("constraint table: many keys, no false positives") {
    // Lots of (cell, t) pairs whose fields are permutations of each other: a weak hash or a
    // lossy key would make some of these collide into wrong answers.
    ConstraintTable table;
    for (int a = 0; a < 20; ++a) {
        for (int b = 0; b < 20; ++b) {
            if (a != b && (a + b) % 3 == 0) table.add(edgeConstraint(0, a, b, 1 + (a * b) % 7));
            if ((a * 7 + b) % 5 == 0) table.add(vertexConstraint(0, a * 20 + b, b));
        }
    }
    for (int a = 0; a < 20; ++a) {
        for (int b = 0; b < 20; ++b) {
            const bool edgeExpected = a != b && (a + b) % 3 == 0;
            for (int t = 1; t <= 7; ++t) {
                CHECK(table.edgeBlocked(a, b, t) == (edgeExpected && t == 1 + (a * b) % 7));
            }
            const bool vertexExpected = (a * 7 + b) % 5 == 0;
            CHECK(table.vertexBlocked(a * 20 + b, b) == vertexExpected);
            CHECK_FALSE(table.vertexBlocked(a * 20 + b, b + 1));
        }
    }
}

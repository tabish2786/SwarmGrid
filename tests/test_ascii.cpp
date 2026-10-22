#include "doctest/doctest.h"

#include <stdexcept>
#include <vector>

#include "swarmgrid/ascii.hpp"

using swarmgrid::Grid;
using swarmgrid::Point;

namespace {

Grid tinyGrid() {
    //  ..@..
    //  ..@..
    //  .....
    Grid g(5, 3);
    g.setObstacle(2, 0, true);
    g.setObstacle(2, 1, true);
    return g;
}

}  // namespace

TEST_CASE("ascii: renderGrid exact output") {
    CHECK(swarmgrid::renderGrid(tinyGrid()) ==
          "..@..\n"
          "..@..\n"
          ".....\n");
}

TEST_CASE("ascii: agentSymbol boundaries") {
    CHECK(swarmgrid::agentSymbol(0) == '0');
    CHECK(swarmgrid::agentSymbol(9) == '9');
    CHECK(swarmgrid::agentSymbol(10) == 'a');
    CHECK(swarmgrid::agentSymbol(35) == 'z');
    CHECK(swarmgrid::agentSymbol(36) == 'A');
    CHECK(swarmgrid::agentSymbol(61) == 'Z');
    CHECK(swarmgrid::agentSymbol(62) == '*');
    CHECK(swarmgrid::agentSymbol(1000) == '*');
}

TEST_CASE("ascii: renderAgents") {
    const Grid g = tinyGrid();
    CHECK(swarmgrid::renderAgents(g, {{0, 0}, {4, 0}}) ==
          "0.@.1\n"
          "..@..\n"
          ".....\n");

    // Collision marker.
    CHECK(swarmgrid::renderAgents(g, {{1, 2}, {3, 2}, {1, 2}}) ==
          "..@..\n"
          "..@..\n"
          ".!.1.\n");

    // Three on one cell is still '!'.
    CHECK(swarmgrid::renderAgents(g, {{0, 0}, {0, 0}, {0, 0}})[0] == '!');

    CHECK_THROWS_AS(swarmgrid::renderAgents(g, {{2, 0}}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::renderAgents(g, {{5, 0}}), std::invalid_argument);
}

TEST_CASE("ascii: renderPath") {
    const Grid g = tinyGrid();
    const std::vector<swarmgrid::CellId> path{
        g.toId(0, 0), g.toId(0, 1), g.toId(0, 2), g.toId(1, 2), g.toId(2, 2),
        g.toId(3, 2), g.toId(3, 1), g.toId(3, 0), g.toId(4, 0)};
    CHECK(swarmgrid::renderPath(g, path) ==
          "S.@+G\n"
          "+.@+.\n"
          "++++.\n");
    CHECK(swarmgrid::renderPath(g, {}) == swarmgrid::renderGrid(g));
    CHECK(swarmgrid::renderPath(g, {g.toId(1, 1)})[7] == 'G');  // row 1 starts at index 6
    CHECK_THROWS_AS(swarmgrid::renderPath(g, {g.toId(2, 0)}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::renderPath(g, {g.numCells()}), std::invalid_argument);
    CHECK_THROWS_AS(swarmgrid::renderPath(g, {-1}), std::invalid_argument);
}

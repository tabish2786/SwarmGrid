#include "doctest/doctest.h"

#include <sstream>
#include <stdexcept>
#include <string>

#include "swarmgrid/plan_io.hpp"

using swarmgrid::Grid;
using swarmgrid::PlanExport;

namespace {

//   . @ .
//   . . .
PlanExport tinyPlan() {
    PlanExport p;
    p.id = "tiny";
    p.title = "Tiny \"demo\"";
    p.solver = "hand-written";
    p.note = "line1\nline2";
    p.grid = Grid(3, 2);
    p.grid.setObstacle(1, 0, true);
    p.starts = {{0, 0}};
    p.goals = {{2, 0}};
    p.paths = {{p.grid.toId(0, 0), p.grid.toId(0, 1), p.grid.toId(1, 1), p.grid.toId(1, 1),
                p.grid.toId(2, 1), p.grid.toId(2, 0)}};
    p.expanded = {7};
    p.constraints = {swarmgrid::vertexConstraint(0, p.grid.toId(2, 1), 3)};
    return p;
}

std::string toJson(const PlanExport& p) {
    std::ostringstream out;
    swarmgrid::writePlanJson(out, p);
    return out.str();
}

}  // namespace

TEST_CASE("plan_io: jsonEscape") {
    CHECK(swarmgrid::jsonEscape("plain") == "plain");
    CHECK(swarmgrid::jsonEscape("a\"b") == "a\\\"b");
    CHECK(swarmgrid::jsonEscape("a\\b") == "a\\\\b");
    CHECK(swarmgrid::jsonEscape("a\nb\tc\rd") == "a\\nb\\tc\\rd");
    CHECK(swarmgrid::jsonEscape(std::string("x\x01y")) == "x\\u0001y");
    CHECK(swarmgrid::jsonEscape(std::string("\x1f")) == "\\u001f");
    CHECK(swarmgrid::jsonEscape("caf\xC3\xA9") == "caf\xC3\xA9");  // UTF-8 passes through
}

TEST_CASE("plan_io: exact JSON for a tiny plan") {
    const std::string expected =
        "{\"format\":\"swarmgrid-plan-v1\",\"id\":\"tiny\",\"title\":\"Tiny \\\"demo\\\"\","
        "\"solver\":\"hand-written\",\"note\":\"line1\\nline2\",\n"
        " \"map\":{\"width\":3,\"height\":2,\"rows\":[\n  \".@.\",\n  \"...\"]},\n"
        " \"agents\":[\n  {\"id\":0,\"start\":[0,0],\"goal\":[2,0],\"cost\":5,\"expanded\":7,"
        "\"path\":[[0,0,0],[0,1,1],[1,1,2],[1,1,3],[2,1,4],[2,0,5]]}],\n"
        " \"constraints\":[\n  {\"agent\":0,\"type\":\"vertex\",\"cell\":[2,1],\"t\":3}],\n"
        " \"stats\":{\"agents\":1,\"sum_of_costs\":5,\"makespan\":5}}";
    CHECK(toJson(tinyPlan()) == expected);
}

TEST_CASE("plan_io: edge constraints, no expansions, several agents") {
    PlanExport p = tinyPlan();
    p.expanded.clear();
    p.starts.push_back({0, 1});
    p.goals.push_back({0, 1});
    p.paths.push_back({p.grid.toId(0, 1)});
    p.constraints = {swarmgrid::edgeConstraint(1, p.grid.toId(0, 1), p.grid.toId(1, 1), 2),
                     swarmgrid::blockedFromConstraint(0, p.grid.toId(2, 0), 5)};
    const std::string json = toJson(p);
    CHECK(json.find("\"expanded\"") == std::string::npos);
    CHECK(json.find("{\"id\":1,\"start\":[0,1],\"goal\":[0,1],\"cost\":0,\"path\":[[0,1,0]]}") !=
          std::string::npos);
    CHECK(json.find("{\"agent\":1,\"type\":\"edge\",\"cell\":[0,1],\"to\":[1,1],\"t\":2}") !=
          std::string::npos);
    CHECK(json.find("{\"agent\":0,\"type\":\"blocked_from\",\"cell\":[2,0],\"t\":5}") !=
          std::string::npos);
    CHECK(json.find("\"stats\":{\"agents\":2,\"sum_of_costs\":5,\"makespan\":5}") !=
          std::string::npos);
}

TEST_CASE("plan_io: inconsistent input throws") {
    std::ostringstream out;
    SUBCASE("size mismatch") {
        PlanExport p = tinyPlan();
        p.goals.clear();
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
    SUBCASE("expanded has the wrong size") {
        PlanExport p = tinyPlan();
        p.expanded = {1, 2};
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
    SUBCASE("empty path") {
        PlanExport p = tinyPlan();
        p.paths[0].clear();
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
    SUBCASE("path cell outside the grid") {
        PlanExport p = tinyPlan();
        p.paths[0].push_back(p.grid.numCells());
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
    SUBCASE("start outside the grid") {
        PlanExport p = tinyPlan();
        p.starts[0] = {3, 0};
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
    SUBCASE("constraint cell outside the grid") {
        PlanExport p = tinyPlan();
        p.constraints = {swarmgrid::vertexConstraint(0, 99, 1)};
        CHECK_THROWS_AS(swarmgrid::writePlanJson(out, p), std::invalid_argument);
    }
}

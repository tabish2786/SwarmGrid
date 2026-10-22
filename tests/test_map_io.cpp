#include "doctest/doctest.h"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarmgrid/map_io.hpp"

using swarmgrid::Grid;
using swarmgrid::Point;
using swarmgrid::ScenarioEntry;

namespace {

Grid parseMap(const std::string& text) {
    std::istringstream in(text);
    return swarmgrid::parseMovingAIMap(in);
}

std::vector<ScenarioEntry> parseScen(const std::string& text) {
    std::istringstream in(text);
    return swarmgrid::parseScenario(in);
}

// 5 wide, 3 tall. Non-square on purpose: catches width/height swaps.
const std::string kTinyMap =
    "type octile\n"
    "height 3\n"
    "width 5\n"
    "map\n"
    "..@..\n"
    ".GT..\n"
    "....@\n";

}  // namespace

TEST_CASE("map: parses dimensions and cells") {
    const Grid g = parseMap(kTinyMap);
    CHECK(g.width() == 5);
    CHECK(g.height() == 3);
    CHECK(g.countObstacles() == 3);
    CHECK_FALSE(g.isFree(2, 0));  // '@'
    CHECK_FALSE(g.isFree(2, 1));  // 'T'
    CHECK_FALSE(g.isFree(4, 2));  // '@'
    CHECK(g.isFree(1, 1));        // 'G' is passable
    CHECK(g.isFree(0, 0));
}

TEST_CASE("map: Windows line endings") {
    std::string crlf;
    for (char ch : kTinyMap) {
        if (ch == '\n') crlf += '\r';
        crlf += ch;
    }
    const Grid g = parseMap(crlf);
    CHECK(g.width() == 5);
    CHECK(g.height() == 3);
    CHECK(g.countObstacles() == 3);
}

TEST_CASE("map: malformed input throws") {
    SUBCASE("missing 'map' line") {
        CHECK_THROWS_AS(parseMap("type octile\nheight 1\nwidth 2\n..\n"), std::runtime_error);
    }
    SUBCASE("row too short") {
        CHECK_THROWS_AS(parseMap("type octile\nheight 2\nwidth 3\nmap\n...\n..\n"),
                        std::runtime_error);
    }
    SUBCASE("row too long") {
        CHECK_THROWS_AS(parseMap("type octile\nheight 1\nwidth 3\nmap\n....\n"),
                        std::runtime_error);
    }
    SUBCASE("too few rows") {
        CHECK_THROWS_AS(parseMap("type octile\nheight 3\nwidth 2\nmap\n..\n..\n"),
                        std::runtime_error);
    }
    SUBCASE("unknown cell character") {
        CHECK_THROWS_AS(parseMap("type octile\nheight 1\nwidth 3\nmap\n.x.\n"),
                        std::runtime_error);
    }
    SUBCASE("non-numeric height") {
        CHECK_THROWS_AS(parseMap("type octile\nheight abc\nwidth 3\nmap\n...\n"),
                        std::runtime_error);
    }
    SUBCASE("empty input") {
        CHECK_THROWS_AS(parseMap(""), std::runtime_error);
    }
}

TEST_CASE("scen: parses tab-separated entries") {
    const auto entries = parseScen(
        "version 1\n"
        "0\ttiny.map\t5\t3\t0\t0\t4\t0\t8\n"
        "1\ttiny.map\t5\t3\t4\t0\t0\t2\t6.82842712\r\n"
        "\n");
    REQUIRE(entries.size() == 2);
    CHECK(entries[0].bucket == 0);
    CHECK(entries[0].mapName == "tiny.map");
    CHECK(entries[0].mapWidth == 5);
    CHECK(entries[0].mapHeight == 3);
    CHECK(entries[0].start == Point{0, 0});
    CHECK(entries[0].goal == Point{4, 0});
    CHECK(entries[1].start == Point{4, 0});
    CHECK(entries[1].goal == Point{0, 2});
    CHECK(entries[1].optimalLength == doctest::Approx(6.82842712));
}

TEST_CASE("scen: malformed input throws") {
    CHECK_THROWS_AS(parseScen("0\ttiny.map\t5\t3\t0\t0\t4\t0\t8\n"), std::runtime_error);
    CHECK_THROWS_AS(parseScen("version 1\n0\ttiny.map\t5\t3\t0\n"), std::runtime_error);
    CHECK_THROWS_AS(parseScen("version 1\n0\ttiny.map\t5\tthree\t0\t0\t4\t0\t8\n"),
                    std::runtime_error);
}

TEST_CASE("makeInstance: takes the first k and validates") {
    const Grid g = parseMap(kTinyMap);
    auto entry = [](Point s, Point t) {
        ScenarioEntry e;
        e.mapName = "tiny.map";
        e.mapWidth = 5;
        e.mapHeight = 3;
        e.start = s;
        e.goal = t;
        return e;
    };
    const std::vector<ScenarioEntry> good{entry({0, 0}, {4, 0}), entry({4, 0}, {0, 0}),
                                          entry({0, 2}, {3, 2})};

    SUBCASE("first k") {
        const auto inst = swarmgrid::makeInstance(g, good, 2);
        CHECK(inst.numAgents() == 2);
        CHECK(inst.starts[1] == Point{4, 0});
        CHECK(inst.goals[1] == Point{0, 0});
        CHECK(inst.grid.width() == 5);
    }
    SUBCASE("bad k") {
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, good, 0), std::invalid_argument);
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, good, 4), std::invalid_argument);
    }
    SUBCASE("start on obstacle") {
        auto bad = good;
        bad[2].start = {2, 0};
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, bad, 3), std::invalid_argument);
    }
    SUBCASE("goal out of bounds") {
        auto bad = good;
        bad[0].goal = {5, 0};
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, bad, 3), std::invalid_argument);
    }
    SUBCASE("duplicate start") {
        auto bad = good;
        bad[2].start = {0, 0};
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, bad, 3), std::invalid_argument);
    }
    SUBCASE("duplicate goal") {
        auto bad = good;
        bad[2].goal = {4, 0};
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, bad, 3), std::invalid_argument);
    }
    SUBCASE("scen dimensions don't match the map") {
        auto bad = good;
        bad[1].mapWidth = 6;
        CHECK_THROWS_AS(swarmgrid::makeInstance(g, bad, 3), std::invalid_argument);
    }
    SUBCASE("duplicates beyond k are ignored") {
        auto ok = good;
        ok[2].start = {0, 0};  // collides with agent 0, but agent 2 is not selected
        CHECK_NOTHROW(swarmgrid::makeInstance(g, ok, 2));
    }
}

TEST_CASE("files: load the bundled tiny map + scen from disk") {
    const std::string dir = SWARMGRID_DATA_DIR;
    const Grid g = swarmgrid::loadMovingAIMap(dir + "/maps/tiny-8x5.map");
    CHECK(g.width() == 8);
    CHECK(g.height() == 5);
    const auto entries = swarmgrid::loadScenario(dir + "/scens/tiny-8x5.scen");
    REQUIRE(entries.size() == 3);
    const auto inst = swarmgrid::makeInstance(g, entries, 3);
    CHECK(inst.numAgents() == 3);

    CHECK_THROWS_AS(swarmgrid::loadMovingAIMap(dir + "/maps/does-not-exist.map"),
                    std::runtime_error);
}

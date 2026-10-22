#pragma once

#include <istream>
#include <string>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

// One line of a MovingAI .scen file.
struct ScenarioEntry {
    int bucket = 0;
    std::string mapName;
    int mapWidth = 0;
    int mapHeight = 0;
    Point start;
    Point goal;
    // WARNING: computed for 8-connected (octile) movement. It will NOT match your
    // 4-connected BFS distance. Never use it to validate M1.
    double optimalLength = 0.0;
};

// A ready-to-solve MAPF problem: agent i goes from starts[i] to goals[i].
struct MapfInstance {
    Grid grid;
    std::vector<Point> starts;
    std::vector<Point> goals;

    int numAgents() const { return static_cast<int>(starts.size()); }
};

// ---- MovingAI .map ----
// Format:
//   type octile
//   height H        <- height comes BEFORE width. Classic bug.
//   width W
//   map
//   <H rows of exactly W chars>
// Free: '.', 'G', 'S'.  Obstacle: '@', 'O', 'T', 'W'.  Any other char -> error.
// Must tolerate Windows line endings ("\r\n").
// Throws std::runtime_error with a useful message on malformed input.
Grid parseMovingAIMap(std::istream& in);
Grid loadMovingAIMap(const std::string& path);  // opens file, calls parseMovingAIMap

// ---- MovingAI .scen ----
// Format:
//   version 1
//   bucket \t map \t map_width \t map_height \t start_x \t start_y \t goal_x \t goal_y \t optimal_length
// Must tolerate "\r\n". Skip empty lines. Throws std::runtime_error on malformed lines.
std::vector<ScenarioEntry> parseScenario(std::istream& in);
std::vector<ScenarioEntry> loadScenario(const std::string& path);

// Takes the FIRST numAgents entries (standard benchmark convention).
// Throws std::invalid_argument if:
//   - numAgents <= 0 or numAgents > entries.size()
//   - an entry's mapWidth/mapHeight don't match the grid
//   - any start/goal is out of bounds or on an obstacle
//   - two agents share a start, or two agents share a goal
MapfInstance makeInstance(const Grid& grid, const std::vector<ScenarioEntry>& entries,
                          int numAgents);

}  // namespace swarmgrid

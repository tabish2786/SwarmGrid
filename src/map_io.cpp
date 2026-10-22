#include "swarmgrid/map_io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace swarmgrid {

namespace {

// Reads one line, strips a trailing '\r' (Windows line endings), and bumps the line counter.
// Returns false at end of input.
bool readLine(std::istream& in, std::string& line, int& lineNumber) {
    if (!std::getline(in, line)) {
        return false;
    }
    ++lineNumber;
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    return true;
}

std::runtime_error parseError(int lineNumber, const std::string& what) {
    return std::runtime_error("line " + std::to_string(lineNumber) + ": " + what);
}

std::string requireLine(std::istream& in, int& lineNumber) {
    std::string line;
    if (!readLine(in, line, lineNumber)) {
        throw parseError(lineNumber + 1, "unexpected end of file");
    }
    return line;
}

// Parses "<key> <positive int>", e.g. "height 32".
int parseHeaderInt(const std::string& line, const std::string& key, int lineNumber) {
    std::istringstream ss(line);
    std::string word;
    int value = 0;
    std::string extra;
    if (!(ss >> word >> value) || word != key || (ss >> extra) || value <= 0) {
        throw parseError(lineNumber, "expected '" + key + " <positive number>', got '" + line + "'");
    }
    return value;
}

bool isPassable(char c) { return c == '.' || c == 'G' || c == 'S'; }
bool isObstacle(char c) { return c == '@' || c == 'O' || c == 'T' || c == 'W'; }

bool isBlank(const std::string& line) {
    return line.find_first_not_of(" \t") == std::string::npos;
}

}  // namespace

Grid parseMovingAIMap(std::istream& in) {
    int lineNumber = 0;

    // Read each line into a variable first: in f(requireLine(in, n), n) the order in which
    // the two arguments are evaluated is unspecified, so n could be read before it is bumped.
    const std::string typeLine = requireLine(in, lineNumber);
    if (typeLine.rfind("type", 0) != 0) {
        throw parseError(lineNumber, "expected 'type ...'");
    }
    const std::string heightLine = requireLine(in, lineNumber);
    const int height = parseHeaderInt(heightLine, "height", lineNumber);
    const std::string widthLine = requireLine(in, lineNumber);
    const int width = parseHeaderInt(widthLine, "width", lineNumber);
    if (requireLine(in, lineNumber) != "map") {
        throw parseError(lineNumber, "expected 'map'");
    }

    Grid grid(width, height);
    for (int y = 0; y < height; ++y) {
        const std::string row = requireLine(in, lineNumber);
        if (static_cast<int>(row.size()) != width) {
            throw parseError(lineNumber, "row has " + std::to_string(row.size()) +
                                             " cells, expected " + std::to_string(width));
        }
        for (int x = 0; x < width; ++x) {
            const char c = row[static_cast<std::size_t>(x)];
            if (isObstacle(c)) {
                grid.setObstacle(x, y, true);
            } else if (!isPassable(c)) {
                throw parseError(lineNumber, std::string("unknown cell character '") + c + "'");
            }
        }
    }
    return grid;
}

Grid loadMovingAIMap(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open map file: " + path);
    }
    return parseMovingAIMap(file);
}

std::vector<ScenarioEntry> parseScenario(std::istream& in) {
    int lineNumber = 0;
    std::string line;
    if (!readLine(in, line, lineNumber) || line.rfind("version", 0) != 0) {
        throw parseError(1, "expected 'version ...' as the first line");
    }

    std::vector<ScenarioEntry> entries;
    while (readLine(in, line, lineNumber)) {
        if (isBlank(line)) {
            continue;
        }
        std::istringstream ss(line);
        ScenarioEntry e;
        std::string extra;
        if (!(ss >> e.bucket >> e.mapName >> e.mapWidth >> e.mapHeight >> e.start.x >>
              e.start.y >> e.goal.x >> e.goal.y >> e.optimalLength) ||
            (ss >> extra)) {
            throw parseError(lineNumber,
                             "expected 9 fields: bucket map width height sx sy gx gy length");
        }
        entries.push_back(e);
    }
    return entries;
}

std::vector<ScenarioEntry> loadScenario(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open scen file: " + path);
    }
    return parseScenario(file);
}

MapfInstance makeInstance(const Grid& grid, const std::vector<ScenarioEntry>& entries,
                          int numAgents) {
    if (numAgents <= 0 || static_cast<std::size_t>(numAgents) > entries.size()) {
        throw std::invalid_argument("makeInstance: need 1.." + std::to_string(entries.size()) +
                                    " agents, got " + std::to_string(numAgents));
    }

    MapfInstance instance;
    instance.grid = grid;
    // Hash sets make the duplicate check O(k) expected instead of O(k^2).
    std::unordered_set<CellId> usedStarts;
    std::unordered_set<CellId> usedGoals;

    for (int i = 0; i < numAgents; ++i) {
        const ScenarioEntry& e = entries[static_cast<std::size_t>(i)];
        const std::string who = "makeInstance: agent " + std::to_string(i) + ": ";
        if (e.mapWidth != grid.width() || e.mapHeight != grid.height()) {
            throw std::invalid_argument(who + "scen map size does not match the grid");
        }
        if (!grid.isFree(e.start.x, e.start.y)) {
            throw std::invalid_argument(who + "start is blocked or outside the grid");
        }
        if (!grid.isFree(e.goal.x, e.goal.y)) {
            throw std::invalid_argument(who + "goal is blocked or outside the grid");
        }
        if (!usedStarts.insert(grid.toId(e.start)).second) {
            throw std::invalid_argument(who + "start is shared with another agent");
        }
        if (!usedGoals.insert(grid.toId(e.goal)).second) {
            throw std::invalid_argument(who + "goal is shared with another agent");
        }
        instance.starts.push_back(e.start);
        instance.goals.push_back(e.goal);
    }
    return instance;
}

}  // namespace swarmgrid
// SwarmGrid CLI.
//
//   swarmgrid map <file.map> [file.scen] [numAgents]
//   swarmgrid warehouse [numAgents] [seed]
//   swarmgrid path <file.map> <file.scen> <agentIndex>
//   swarmgrid astar-bench [numQueries] [seed]
//
// Later milestones add: solve, validate, bench, lifelong.

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "swarmgrid/ascii.hpp"
#include "swarmgrid/astar.hpp"
#include "swarmgrid/grid.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/map_io.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace {

void printUsage() {
    std::cerr << "usage:\n"
              << "  swarmgrid map <file.map> [file.scen] [numAgents]\n"
              << "  swarmgrid warehouse [numAgents] [seed]\n"
              << "  swarmgrid path <file.map> <file.scen> <agentIndex>\n"
              << "  swarmgrid astar-bench [numQueries] [seed]\n";
}

// Strict integer argument: the whole text must be a number, otherwise an error that names the
// argument. std::stoi alone accepts "12abc" and its error text differs between std libraries.
int parseIntArg(const char* text, const char* name) {
    const std::string s = text;
    std::size_t pos = 0;
    int value = 0;
    try {
        value = std::stoi(s, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (pos == 0 || pos != s.size()) {
        throw std::invalid_argument(std::string(name) + ": expected an integer, got '" + s + "'");
    }
    return value;
}

// Seeds must parse identically everywhere. std::stoul is 32-bit on Windows but 64-bit on Linux,
// so stoul + cast would turn one command line into different seeds (or an error) per platform.
std::uint32_t parseSeedArg(const char* text) {
    const std::string s = text;
    const bool digitsOnly = !s.empty() && s.size() <= 10 &&
                            s.find_first_not_of("0123456789") == std::string::npos;
    if (!digitsOnly || std::stoull(s) > 0xFFFFFFFFULL) {
        throw std::invalid_argument("seed: expected an integer in 0..4294967295, got '" + s + "'");
    }
    return static_cast<std::uint32_t>(std::stoull(s));
}

void printInstance(const swarmgrid::MapfInstance& inst) {
    std::cout << "starts:\n" << swarmgrid::renderAgents(inst.grid, inst.starts);
    std::cout << "goals:\n" << swarmgrid::renderAgents(inst.grid, inst.goals);
}

int runMap(int argc, char** argv) {
    if (argc < 3) {
        printUsage();
        return 1;
    }
    const swarmgrid::Grid grid = swarmgrid::loadMovingAIMap(argv[2]);
    std::cout << grid.width() << " x " << grid.height() << ", " << grid.countObstacles()
              << " obstacles\n"
              << swarmgrid::renderGrid(grid);
    if (argc >= 4) {
        const auto entries = swarmgrid::loadScenario(argv[3]);
        const int k =
            argc >= 5 ? parseIntArg(argv[4], "numAgents") : static_cast<int>(entries.size());
        printInstance(swarmgrid::makeInstance(grid, entries, k));
    }
    return 0;
}

int runWarehouse(int argc, char** argv) {
    const int k = argc >= 3 ? parseIntArg(argv[2], "numAgents") : 0;
    const std::uint32_t seed = argc >= 4 ? parseSeedArg(argv[3]) : 42u;
    const swarmgrid::Grid grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    std::cout << grid.width() << " x " << grid.height() << ", " << grid.countObstacles()
              << " obstacles\n"
              << swarmgrid::renderGrid(grid);
    if (k > 0) {
        printInstance(swarmgrid::generateRandomInstance(grid, k, seed));
    }
    return 0;
}

// Shortest path for one agent of a scenario, with both heuristics side by side.
int runPath(int argc, char** argv) {
    if (argc < 5) {
        printUsage();
        return 1;
    }
    const swarmgrid::Grid grid = swarmgrid::loadMovingAIMap(argv[2]);
    const auto entries = swarmgrid::loadScenario(argv[3]);
    const int agent = parseIntArg(argv[4], "agentIndex");
    if (agent < 0 || agent >= static_cast<int>(entries.size())) {
        throw std::invalid_argument("agentIndex must be in 0.." +
                                    std::to_string(entries.size() - 1));
    }
    // Validate only this entry: a single-agent query doesn't care whether other entries share
    // its start or goal.
    const swarmgrid::MapfInstance inst =
        swarmgrid::makeInstance(grid, {entries[static_cast<std::size_t>(agent)]}, 1);
    const swarmgrid::CellId start = grid.toId(inst.starts[0]);
    const swarmgrid::CellId goal = grid.toId(inst.goals[0]);

    swarmgrid::SearchStats bfsStats;
    swarmgrid::SearchStats manStats;
    const auto path =
        swarmgrid::astar(grid, start, goal, swarmgrid::bfsDistances(grid, goal), &bfsStats);
    const auto manPath =
        swarmgrid::astar(grid, start, goal, swarmgrid::manhattanDistances(grid, goal), &manStats);
    if (path.has_value() != manPath.has_value() || (path && path->size() != manPath->size())) {
        throw std::logic_error("heuristics disagree on the optimal path (bug in astar/heuristic)");
    }
    if (!path) {
        std::cout << "no path from start to goal\n";
        return 0;
    }
    std::cout << swarmgrid::renderPath(grid, *path) << "moves: " << path->size() - 1 << '\n'
              << "expanded with BFS heuristic:       " << bfsStats.expanded << '\n'
              << "expanded with Manhattan heuristic: " << manStats.expanded << '\n';
    return 0;
}

// Same random queries on the generated warehouse, solved with both heuristics.
// Expansion counts are deterministic, so they are identical on every machine.
int runAstarBench(int argc, char** argv) {
    const int k = argc >= 3 ? parseIntArg(argv[2], "numQueries") : 100;
    const std::uint32_t seed = argc >= 4 ? parseSeedArg(argv[3]) : 42u;
    const swarmgrid::Grid grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const int freeCells = grid.numCells() - grid.countObstacles();
    if (k < 1 || k > freeCells) {
        // Queries come from generateRandomInstance, which needs distinct starts and goals.
        throw std::invalid_argument("numQueries must be in 1.." + std::to_string(freeCells));
    }
    const swarmgrid::MapfInstance inst = swarmgrid::generateRandomInstance(grid, k, seed);

    long long bfsExpanded = 0;
    long long manExpanded = 0;
    long long totalMoves = 0;
    long long bfsTableWork = 0;  // cells visited while building the BFS tables
    for (std::size_t i = 0; i < inst.starts.size(); ++i) {
        const swarmgrid::CellId s = grid.toId(inst.starts[i]);
        const swarmgrid::CellId t = grid.toId(inst.goals[i]);
        swarmgrid::SearchStats a;
        swarmgrid::SearchStats b;
        const std::vector<int> table = swarmgrid::bfsDistances(grid, t);
        for (int d : table) {
            if (d != swarmgrid::kUnreachable) ++bfsTableWork;  // each reached cell is visited once
        }
        const auto p1 = swarmgrid::astar(grid, s, t, table, &a);
        const auto p2 = swarmgrid::astar(grid, s, t, swarmgrid::manhattanDistances(grid, t), &b);
        if (!p1 || !p2 || p1->size() != p2->size()) {
            throw std::logic_error("heuristics disagree on the optimal path length");
        }
        bfsExpanded += a.expanded;
        manExpanded += b.expanded;
        totalMoves += static_cast<long long>(p1->size()) - 1;
    }
    // An exact heuristic expands the minimum possible (moves + 1 per query), so this holds.
    if (bfsExpanded > manExpanded) {
        throw std::logic_error("BFS heuristic expanded more than Manhattan (bug)");
    }
    // Percentage in tenths, rounded half up with integer math so the printed text is identical
    // on every C runtime (printf rounding of doubles differs between glibc and old Windows CRTs).
    const long long tenths = (2000 * (manExpanded - bfsExpanded) + manExpanded) / (2 * manExpanded);
    std::cout << "map: generated warehouse " << grid.width() << " x " << grid.height()
              << ", queries: " << k << ", seed: " << seed << '\n'
              << "total optimal moves:               " << totalMoves << '\n'
              << "expanded with Manhattan heuristic: " << manExpanded << '\n'
              << "expanded with BFS heuristic:       " << bfsExpanded
              << "  (= moves + queries: an exact heuristic walks one optimal path)\n"
              << "expansion reduction:               " << tenths / 10 << '.' << tenths % 10 << "%\n"
              << "BFS table build work:              " << bfsTableWork << " cell visits\n"
              << "note: every query here has a different goal, so each BFS table is used once and\n"
              << "      costs more than it saves. Tables pay off when one goal is searched many\n"
              << "      times (e.g. CBS replanning the same agent).\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }
    try {
        const std::string cmd = argv[1];
        if (cmd == "map") return runMap(argc, argv);
        if (cmd == "warehouse") return runWarehouse(argc, argv);
        if (cmd == "path") return runPath(argc, argv);
        if (cmd == "astar-bench") return runAstarBench(argc, argv);
        printUsage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}

// SwarmGrid CLI.
//
//   swarmgrid map <file.map> [file.scen] [numAgents]
//   swarmgrid warehouse [numAgents] [seed]
//   swarmgrid path <file.map> <file.scen> <agentIndex>
//   swarmgrid astar-bench [numQueries] [seed]
//   swarmgrid st-demo
//   swarmgrid export-plan <file.map> <file.scen> <numAgents> <out.json>
//   swarmgrid export-demos <out.js>
//   swarmgrid cbs-bench [timeLimitSeconds]
//   swarmgrid bench --out results.csv [options]            (M7, see bench_cli.hpp)
//   swarmgrid lifelong [options]                            (M8)
//   swarmgrid lifelong-bench --out lifelong.csv [options]   (M8)

#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "bench_cli.hpp"
#include "swarmgrid/ascii.hpp"
#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/astar.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/grid.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/map_io.hpp"
#include "swarmgrid/plan_io.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/space_time_astar.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace {

void printUsage() {
    std::cerr << "usage:\n"
              << "  swarmgrid map <file.map> [file.scen] [numAgents]\n"
              << "  swarmgrid warehouse [numAgents] [seed]\n"
              << "  swarmgrid path <file.map> <file.scen> <agentIndex>\n"
              << "  swarmgrid astar-bench [numQueries] [seed]\n"
              << "  swarmgrid st-demo\n"
              << "  swarmgrid export-plan <file.map> <file.scen> <numAgents> <out.json>\n"
              << "  swarmgrid export-demos <out.js>\n"
              << "  swarmgrid cbs-bench [timeLimitSeconds]\n"
              << "  swarmgrid bench --out results.csv [options]   (--help for options)\n"
              << "  swarmgrid lifelong [options]                  (--help for options)\n"
              << "  swarmgrid lifelong-bench --out lifelong.csv [options]\n";
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

struct BenchResult {
    int queries = 0;
    std::uint32_t seed = 0;
    int width = 0;
    int height = 0;
    long long totalMoves = 0;
    long long manExpanded = 0;
    long long bfsExpanded = 0;
    long long bfsTableWork = 0;  // cells visited while building the BFS tables
};

// Same random queries on the generated warehouse, solved with both heuristics.
// Expansion counts are deterministic, so they are identical on every machine.
BenchResult computeAstarBench(int k, std::uint32_t seed) {
    const swarmgrid::Grid grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const int freeCells = grid.numCells() - grid.countObstacles();
    if (k < 1 || k > freeCells) {
        // Queries come from generateRandomInstance, which needs distinct starts and goals.
        throw std::invalid_argument("numQueries must be in 1.." + std::to_string(freeCells));
    }
    const swarmgrid::MapfInstance inst = swarmgrid::generateRandomInstance(grid, k, seed);

    BenchResult r;
    r.queries = k;
    r.seed = seed;
    r.width = grid.width();
    r.height = grid.height();
    long long& bfsExpanded = r.bfsExpanded;
    long long& manExpanded = r.manExpanded;
    long long& totalMoves = r.totalMoves;
    long long& bfsTableWork = r.bfsTableWork;
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
    return r;
}

int runAstarBench(int argc, char** argv) {
    const int k = argc >= 3 ? parseIntArg(argv[2], "numQueries") : 100;
    const std::uint32_t seed = argc >= 4 ? parseSeedArg(argv[3]) : 42u;
    const BenchResult r = computeAstarBench(k, seed);
    const long long manExpanded = r.manExpanded;
    const long long bfsExpanded = r.bfsExpanded;
    // Percentage in tenths, rounded half up with integer math so the printed text is identical
    // on every C runtime (printf rounding of doubles differs between glibc and old Windows CRTs).
    const long long tenths = (2000 * (manExpanded - bfsExpanded) + manExpanded) / (2 * manExpanded);
    std::cout << "map: generated warehouse " << r.width << " x " << r.height
              << ", queries: " << k << ", seed: " << seed << '\n'
              << "total optimal moves:               " << r.totalMoves << '\n'
              << "expanded with Manhattan heuristic: " << manExpanded << '\n'
              << "expanded with BFS heuristic:       " << bfsExpanded
              << "  (= moves + queries: an exact heuristic walks one optimal path)\n"
              << "expansion reduction:               " << tenths / 10 << '.' << tenths % 10 << "%\n"
              << "BFS table build work:              " << r.bfsTableWork << " cell visits\n"
              << "note: every query here has a different goal, so each BFS table is used once and\n"
              << "      costs more than it saves. Tables pay off when one goal is searched many\n"
              << "      times (e.g. CBS replanning the same agent).\n";
    return 0;
}

// M2 demo: one corridor, four constraint sets, and how each one changes the timed path.
int runSpaceTimeDemo() {
    const swarmgrid::Grid grid(5, 1);  // S . . . G
    const swarmgrid::CellId start = grid.toId(0, 0);
    const swarmgrid::CellId goal = grid.toId(4, 0);
    const std::vector<int> h = swarmgrid::bfsDistances(grid, goal);

    struct Case {
        const char* label;
        std::vector<swarmgrid::Constraint> constraints;
    };
    const std::vector<Case> cases = {
        {"no constraints            ", {}},
        {"vertex: not (2,0) at t=2  ", {swarmgrid::vertexConstraint(0, grid.toId(2, 0), 2)}},
        {"edge: not (0,0)->(1,0) t=1",
         {swarmgrid::edgeConstraint(0, grid.toId(0, 0), grid.toId(1, 0), 1)}},
        {"goal: not (4,0) at t=6    ", {swarmgrid::vertexConstraint(0, goal, 6)}},
    };

    std::cout << "corridor 5 x 1, start (0,0), goal (4,0)\n";
    for (const Case& c : cases) {
        const swarmgrid::ConstraintTable table(c.constraints, 0);
        const auto path = swarmgrid::spaceTimeAstar(grid, start, goal, h, table);
        std::cout << c.label << "  ";
        if (!path) {
            std::cout << "no path\n";
            continue;
        }
        std::cout << "arrival " << path->size() - 1 << ": "
                  << swarmgrid::formatTimedPath(grid, *path) << '\n';
    }
    return 0;
}

// Every agent planned on its own with A* + BFS heuristic: no coordination, so paths may collide.
swarmgrid::PlanExport independentPlan(const swarmgrid::MapfInstance& inst, const std::string& id,
                                      const std::string& title) {
    swarmgrid::PlanExport plan;
    plan.id = id;
    plan.title = title;
    plan.solver = "Independent A* per robot (BFS heuristic). No coordination yet.";
    plan.note =
        "Each robot is planned on its own, so the robots don't know about each other yet. "
        "Red rings mark collisions. Conflict-Based Search (milestone M4) will remove them.";
    plan.grid = inst.grid;
    plan.starts = inst.starts;
    plan.goals = inst.goals;
    for (std::size_t i = 0; i < inst.starts.size(); ++i) {
        const swarmgrid::CellId s = inst.grid.toId(inst.starts[i]);
        const swarmgrid::CellId t = inst.grid.toId(inst.goals[i]);
        swarmgrid::SearchStats stats;
        const auto path = swarmgrid::astar(inst.grid, s, t, swarmgrid::bfsDistances(inst.grid, t),
                                           &stats);
        if (!path) {
            throw std::runtime_error("agent " + std::to_string(i) + " cannot reach its goal");
        }
        plan.paths.push_back(*path);
        plan.expanded.push_back(stats.expanded);
    }
    return plan;
}

// Same instance, planned with prioritized planning: collision-free, slightly longer paths.
swarmgrid::PlanExport prioritizedPlan(const swarmgrid::MapfInstance& inst, const std::string& id,
                                      const std::string& title) {
    std::vector<swarmgrid::CellId> starts;
    std::vector<swarmgrid::CellId> goals;
    for (std::size_t i = 0; i < inst.starts.size(); ++i) {
        starts.push_back(inst.grid.toId(inst.starts[i]));
        goals.push_back(inst.grid.toId(inst.goals[i]));
    }
    const swarmgrid::MultiAgentResult r = swarmgrid::planMultiAgent(
        inst.grid, starts, goals, swarmgrid::MultiAgentMode::Prioritized);
    if (!r.solved) throw std::runtime_error("prioritized planning failed for " + id);
    swarmgrid::PlanExport plan;
    plan.id = id;
    plan.title = title;
    plan.solver = "Prioritized planning: robots planned one by one with space-time A*.";
    plan.note =
        "Same robots as the independent plan, but each robot avoids the ones planned before it "
        "(their cells, their moves, and their parked goals). Zero collisions, at the price of a few "
        "extra timesteps. Not optimal and can fail on hard instances; CBS (M4) will be optimal.";
    plan.grid = inst.grid;
    plan.starts = inst.starts;
    plan.goals = inst.goals;
    plan.paths = r.paths;
    return plan;
}

std::vector<swarmgrid::CellId> cellIds(const swarmgrid::Grid& g, const std::vector<swarmgrid::Point>& pts) {
    std::vector<swarmgrid::CellId> out;
    for (const swarmgrid::Point& p : pts) out.push_back(g.toId(p));
    return out;
}

// Same instance, solved optimally with CBS and checked by the independent validator.
swarmgrid::PlanExport cbsPlan(const swarmgrid::MapfInstance& inst, const std::string& id,
                              const std::string& title) {
    const auto starts = cellIds(inst.grid, inst.starts);
    const auto goals = cellIds(inst.grid, inst.goals);
    const swarmgrid::CbsResult r = swarmgrid::solveCbs(inst.grid, starts, goals);
    if (!r.solved || !swarmgrid::validatePlan(inst.grid, starts, goals, r.paths).ok) {
        throw std::runtime_error("CBS failed or produced an invalid plan for " + id);
    }
    swarmgrid::CbsOptions plainOptions;
    plainOptions.prioritizeConflicts = false;
    plainOptions.bypass = false;
    const swarmgrid::CbsResult plain = swarmgrid::solveCbs(inst.grid, starts, goals, plainOptions);
    swarmgrid::PlanExport plan;
    plan.id = id;
    plan.title = title;
    plan.solver = "Conflict-Based Search (optimal sum of costs), checked by the independent validator.";
    plan.note =
        "Same robots again, now solved optimally with CBS: zero collisions and the lowest possible "
        "sum of costs (" + std::to_string(r.sumOfCosts) + ", vs 212 for prioritized planning). CBS explored " +
        std::to_string(r.nodesExpanded) + " constraint-tree nodes to prove it; without the M5 speedups "
        "(MDD conflict prioritization and bypass) it needed " + std::to_string(plain.nodesExpanded) + ".";
    plan.grid = inst.grid;
    plan.starts = inst.starts;
    plan.goals = inst.goals;
    plan.paths = r.paths;
    return plan;
}

// M6 showcase: 50 robots, too many for CBS on this small map, solved by ECBS within w of optimal.
swarmgrid::PlanExport ecbsPlan(const swarmgrid::MapfInstance& inst, double weight, const std::string& id,
                               const std::string& title) {
    const auto starts = cellIds(inst.grid, inst.starts);
    const auto goals = cellIds(inst.grid, inst.goals);
    swarmgrid::EcbsOptions o;
    o.weight = weight;
    const swarmgrid::EcbsResult r = swarmgrid::solveEcbs(inst.grid, starts, goals, o);
    if (!r.solved || !swarmgrid::validatePlan(inst.grid, starts, goals, r.paths).ok) {
        throw std::runtime_error("ECBS failed or produced an invalid plan for " + id);
    }
    const swarmgrid::MultiAgentResult pri =
        swarmgrid::planMultiAgent(inst.grid, starts, goals, swarmgrid::MultiAgentMode::Prioritized);
    long long priCost = 0;
    for (const auto& p : pri.paths) priCost += static_cast<long long>(p.size()) - 1;
    swarmgrid::PlanExport plan;
    plan.id = id;
    plan.title = title;
    plan.solver = "ECBS (bounded-suboptimal CBS with focal search), checked by the independent validator.";
    std::ostringstream w;
    w << weight;
    // The CBS figure was measured once (CBS + M5, 60 s limit, 89,893 nodes, unsolved); it is not
    // rerun here because that would make export-demos take a minute.
    plan.note = "50 robots on the small warehouse: CBS (with the M5 speedups) had not finished after 60 s "
                "when we tried. ECBS with w = " + w.str() + " finishes in " + std::to_string(r.nodesExpanded) + " constraint-tree nodes. Sum of costs " +
                std::to_string(r.sumOfCosts) + "; no plan can be below " + std::to_string(r.lowerBound) +
                ", so it is provably within w of optimal." +
                (pri.solved ? " Prioritized planning needs " + std::to_string(priCost) + "." : "");
    plan.grid = inst.grid;
    plan.starts = inst.starts;
    plan.goals = inst.goals;
    plan.paths = r.paths;
    return plan;
}

// M2 showcase: four robots, each in its own corridor with its own constraints.
swarmgrid::PlanExport constraintsPlan() {
    swarmgrid::PlanExport plan;
    plan.id = "constraints";
    plan.title = "Space-time A* with constraints";
    plan.solver = "Space-time A* per robot (M2), each with its own constraint table.";
    plan.note =
        "Each robot has its own corridor and its own constraints. Robot 0: none. Robot 1: may "
        "not be on (2,2) at t=2. Robot 2: may not move (0,4)->(1,4) at t=1. Robot 3: may not be "
        "on its goal (4,6) at t=6. Watch them wait instead of breaking a constraint.";
    plan.grid = swarmgrid::Grid(5, 7);  // corridors on rows 0, 2, 4, 6; walls in between
    for (int y = 1; y < 7; y += 2) {
        for (int x = 0; x < 5; ++x) plan.grid.setObstacle(x, y, true);
    }
    const swarmgrid::Grid& g = plan.grid;
    plan.constraints = {
        swarmgrid::vertexConstraint(1, g.toId(2, 2), 2),
        swarmgrid::edgeConstraint(2, g.toId(0, 4), g.toId(1, 4), 1),
        swarmgrid::vertexConstraint(3, g.toId(4, 6), 6),
    };
    for (int i = 0; i < 4; ++i) {
        const swarmgrid::Point s{0, 2 * i};
        const swarmgrid::Point t{4, 2 * i};
        const swarmgrid::ConstraintTable table(plan.constraints, i);
        swarmgrid::SearchStats stats;
        const auto path = swarmgrid::spaceTimeAstar(g, g.toId(s), g.toId(t),
                                                    swarmgrid::bfsDistances(g, g.toId(t)), table,
                                                    -1, &stats);
        if (!path) throw std::logic_error("constraint demo: no path");
        plan.starts.push_back(s);
        plan.goals.push_back(t);
        plan.paths.push_back(*path);
        plan.expanded.push_back(stats.expanded);
    }
    return plan;
}

// Binary mode: identical bytes (LF line endings) on every OS.
std::ofstream openOutput(const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path);
    return out;
}

int runExportPlan(int argc, char** argv) {
    if (argc < 6) {
        printUsage();
        return 1;
    }
    const swarmgrid::Grid grid = swarmgrid::loadMovingAIMap(argv[2]);
    const auto entries = swarmgrid::loadScenario(argv[3]);
    const int k = parseIntArg(argv[4], "numAgents");
    const swarmgrid::MapfInstance inst = swarmgrid::makeInstance(grid, entries, k);
    const std::string mapName = entries.empty() ? std::string("map") : entries.front().mapName;
    const swarmgrid::PlanExport plan =
        independentPlan(inst, "export", mapName + ", " + std::to_string(k) + " robots");
    std::ofstream out = openOutput(argv[5]);
    swarmgrid::writePlanJson(out, plan);
    out << '\n';
    std::cout << "wrote " << argv[5] << '\n';
    return 0;
}

// The data file the dashboard (docs/index.html) loads. Regenerate after changing the planner:
//   swarmgrid export-demos docs/data.js
int runExportDemos(int argc, char** argv) {
    if (argc < 3) {
        printUsage();
        return 1;
    }
    const swarmgrid::Grid warehouse = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    const swarmgrid::MapfInstance ten = swarmgrid::generateRandomInstance(warehouse, 10, 42);
    const std::vector<swarmgrid::PlanExport> plans = {
        independentPlan(ten, "warehouse-10", "Warehouse, 10 robots (seed 42)"),
        prioritizedPlan(ten, "warehouse-10-prioritized",
                        "Warehouse, 10 robots, collision-free (prioritized)"),
        cbsPlan(ten, "warehouse-10-cbs", "Warehouse, 10 robots, optimal (CBS)"),
        independentPlan(swarmgrid::generateRandomInstance(warehouse, 30, 7), "warehouse-30",
                        "Warehouse, 30 robots (seed 7)"),
        ecbsPlan(swarmgrid::generateRandomInstance(warehouse, 50, 42), 1.5, "warehouse-50-ecbs",
                 "Warehouse, 50 robots, bounded (ECBS w = 1.5)"),
        constraintsPlan(),
    };
    const BenchResult b = computeAstarBench(100, 42);

    std::ofstream out = openOutput(argv[2]);
    out << "// Generated by `swarmgrid export-demos`. Do not edit by hand.\n"
        << "window.SWARMGRID_DATA = {\n\"bench\":{\"queries\":" << b.queries
        << ",\"seed\":" << b.seed << ",\"width\":" << b.width << ",\"height\":" << b.height
        << ",\"total_moves\":" << b.totalMoves << ",\"expanded_manhattan\":" << b.manExpanded
        << ",\"expanded_bfs\":" << b.bfsExpanded << ",\"bfs_table_work\":" << b.bfsTableWork
        << "},\n\"plans\":[\n";
    for (std::size_t i = 0; i < plans.size(); ++i) {
        if (i > 0) out << ",\n";
        swarmgrid::writePlanJson(out, plans[i]);
    }
    out << "\n]};\n";
    std::cout << "wrote " << argv[2] << " (" << plans.size() << " plans)\n";
    return 0;
}

// CBS vs prioritized planning on the generated warehouse: 10 random instances per fleet size.
// Success and costs are deterministic; times depend on the machine.
int runCbsBench(int argc, char** argv) {
    double limit = 2.0;
    if (argc >= 3) {
        limit = std::stod(argv[2]);
        if (!(limit > 0)) throw std::invalid_argument("timeLimitSeconds must be > 0");
    }
    const swarmgrid::Grid g = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    std::cout << "generated warehouse " << g.width() << " x " << g.height() << ", 10 instances per row, "
              << "time limit " << limit << " s per solve\n"
              << "plain = M4 CBS, M5 = CBS + MDD conflict prioritization + bypass (both optimal)\n"
              << "robots | solved plain / M5 | CT nodes expanded plain -> M5 (solved by both) | "
                 "mean time plain -> M5 (s) | prioritized solved | M5 cost vs prioritized\n";
    for (int n : {5, 10, 15, 20, 25, 30}) {
        int plainSolved = 0, m5Solved = 0, both = 0, priSolved = 0, priBoth = 0;
        double plainTime = 0, m5Time = 0;
        long long plainNodes = 0, m5Nodes = 0, m5Cost = 0, priCost = 0;
        for (std::uint32_t seed = 1; seed <= 10; ++seed) {
            const swarmgrid::MapfInstance inst = swarmgrid::generateRandomInstance(g, n, seed);
            const auto s = cellIds(g, inst.starts);
            const auto t = cellIds(g, inst.goals);
            swarmgrid::CbsOptions opts;
            opts.timeLimitSeconds = limit;
            opts.maxNodes = 1000000;
            const swarmgrid::CbsResult m5 = swarmgrid::solveCbs(g, s, t, opts);
            opts.prioritizeConflicts = false;
            opts.bypass = false;
            const swarmgrid::CbsResult plain = swarmgrid::solveCbs(g, s, t, opts);
            const swarmgrid::MultiAgentResult p =
                swarmgrid::planMultiAgent(g, s, t, swarmgrid::MultiAgentMode::Prioritized);
            for (const auto* r : {&plain, &m5}) {
                if (r->solved && !swarmgrid::validatePlan(g, s, t, r->paths).ok) {
                    throw std::logic_error("invalid CBS plan");
                }
            }
            if (plain.solved && m5.solved && plain.sumOfCosts != m5.sumOfCosts) {
                throw std::logic_error("plain CBS and M5 disagree on the optimal cost");
            }
            plainSolved += plain.solved ? 1 : 0;
            m5Solved += m5.solved ? 1 : 0;
            priSolved += p.solved ? 1 : 0;
            if (plain.solved && m5.solved) {
                ++both;
                plainNodes += plain.nodesExpanded;
                m5Nodes += m5.nodesExpanded;
                plainTime += plain.seconds;
                m5Time += m5.seconds;
            }
            if (m5.solved && p.solved) {
                ++priBoth;
                m5Cost += m5.sumOfCosts;
                for (const auto& path : p.paths) priCost += static_cast<long long>(path.size()) - 1;
            }
        }
        std::cout << n << " | " << plainSolved << " / " << m5Solved << " of 10 | ";
        if (both) {
            std::cout << plainNodes << " -> " << m5Nodes << " (" << static_cast<double>(plainNodes) / static_cast<double>(m5Nodes > 0 ? m5Nodes : 1)
                      << "x fewer) | " << plainTime / both << " -> " << m5Time / both;
        } else {
            std::cout << "- | -";
        }
        std::cout << " | " << priSolved << "/10 | ";
        if (priBoth) std::cout << m5Cost << " vs " << priCost;
        std::cout << '\n';
    }
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
        if (cmd == "st-demo") return runSpaceTimeDemo();
        if (cmd == "export-plan") return runExportPlan(argc, argv);
        if (cmd == "export-demos") return runExportDemos(argc, argv);
        if (cmd == "cbs-bench") return runCbsBench(argc, argv);
        if (cmd == "bench") return runBench(argc, argv);
        if (cmd == "lifelong") return runLifelongCommand(argc, argv);
        if (cmd == "lifelong-bench") return runLifelongBench(argc, argv);
        printUsage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}

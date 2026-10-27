#include "bench_cli.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/lifelong.hpp"
#include "swarmgrid/map_io.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace {

// --key value options. Unknown keys are an error, so a typo never silently runs a default.
class Args {
public:
    Args(int argc, char** argv, int first, std::vector<std::string> known) : known_(std::move(known)) {
        for (int i = first; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                help_ = true;
                continue;
            }
            if (key.rfind("--", 0) != 0 || std::find(known_.begin(), known_.end(), key.substr(2)) == known_.end()) {
                throw std::invalid_argument("unknown option '" + key + "' (try --help)");
            }
            if (i + 1 >= argc) throw std::invalid_argument("option " + key + " needs a value");
            values_[key.substr(2)] = argv[++i];
        }
    }
    bool help() const { return help_; }
    bool has(const std::string& k) const { return values_.count(k) > 0; }
    std::string str(const std::string& k, const std::string& def) const {
        auto it = values_.find(k);
        return it == values_.end() ? def : it->second;
    }
    int integer(const std::string& k, int def) const {
        if (!has(k)) return def;
        return parseInt(values_.at(k), k);
    }
    double real(const std::string& k, double def) const {
        if (!has(k)) return def;
        const std::string& s = values_.at(k);
        std::size_t pos = 0;
        double v = 0;
        try {
            v = std::stod(s, &pos);
        } catch (const std::exception&) {
            pos = 0;
        }
        if (pos == 0 || pos != s.size()) throw std::invalid_argument("--" + k + ": expected a number, got '" + s + "'");
        return v;
    }
    std::vector<int> intList(const std::string& k, const std::vector<int>& def) const {
        if (!has(k)) return def;
        std::vector<int> out;
        for (const std::string& part : split(values_.at(k))) out.push_back(parseInt(part, k));
        return out;
    }
    std::vector<std::string> strList(const std::string& k, const std::vector<std::string>& def) const {
        return has(k) ? split(values_.at(k)) : def;
    }

    static std::vector<std::string> split(const std::string& s) {
        std::vector<std::string> out;
        std::stringstream ss(s);
        std::string part;
        while (std::getline(ss, part, ',')) {
            if (!part.empty()) out.push_back(part);
        }
        return out;
    }

private:
    static int parseInt(const std::string& s, const std::string& k) {
        std::size_t pos = 0;
        int v = 0;
        try {
            v = std::stoi(s, &pos);
        } catch (const std::exception&) {
            pos = 0;
        }
        if (pos == 0 || pos != s.size()) throw std::invalid_argument("--" + k + ": expected an integer, got '" + s + "'");
        return v;
    }

    std::vector<std::string> known_;
    std::map<std::string, std::string> values_;
    bool help_ = false;
};

// "small": the default warehouse used everywhere else (27 x 21).
// "large": 8 x 8 blocks of 10 x 2 shelves with 2-wide aisles, 100 x 36, 2320 free cells. A
// generated stand-in for a MovingAI warehouse map when those files are not at hand.
swarmgrid::Grid generatedMap(const std::string& name) {
    if (name == "small") return swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    if (name == "large") {
        swarmgrid::WarehouseParams p;
        p.blocksX = 8;
        p.blocksY = 8;
        p.shelfBlockWidth = 10;
        p.shelfBlockHeight = 2;
        p.aisleWidth = 2;
        p.border = 3;
        return swarmgrid::generateWarehouse(p);
    }
    throw std::invalid_argument("--generated: expected small or large, got '" + name + "'");
}

std::vector<swarmgrid::CellId> ids(const swarmgrid::Grid& g, const std::vector<swarmgrid::Point>& pts) {
    std::vector<swarmgrid::CellId> out;
    for (const auto& p : pts) out.push_back(g.toId(p));
    return out;
}

// MovingAI scenario files of one map, in the order of their number: <map>-random-1.scen, ...
// The folder may hold the scenarios of every map (MovingAI's scen-random.zip does), so only
// files whose name starts with "<map>-" count.
std::vector<std::string> scenarioFiles(const std::string& dir, const std::string& mapStem) {
    std::vector<std::pair<int, std::string>> found;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (entry.path().extension() != ".scen" || name.rfind(mapStem + "-", 0) != 0) continue;
        const auto dash = name.find_last_of('-');
        const auto dot = name.find_last_of('.');
        int number = 0;
        try {
            number = std::stoi(name.substr(dash + 1, dot - dash - 1));
        } catch (const std::exception&) {
            number = 1000000;  // no number: after the numbered ones, by name
        }
        found.emplace_back(number, entry.path().string());
    }
    std::sort(found.begin(), found.end());
    std::vector<std::string> out;
    for (auto& f : found) out.push_back(f.second);
    if (out.empty()) throw std::invalid_argument("no " + mapStem + "-*.scen files in " + dir);
    return out;
}

struct Row {
    std::string algorithm;
    bool solved = false;
    bool timeout = false;
    long long soc = 0;
    int makespan = 0;
    long long lowerBound = 0;
    double seconds = 0;
    long long ctExpanded = 0;
    long long ctGenerated = 0;
    long long lowLevel = 0;
};

Row runAlgorithm(const std::string& algo, const swarmgrid::Grid& g, const std::vector<swarmgrid::CellId>& s,
                 const std::vector<swarmgrid::CellId>& t, double timeout) {
    Row row;
    row.algorithm = algo;
    if (algo == "pp") {
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = swarmgrid::planMultiAgent(g, s, t, swarmgrid::MultiAgentMode::Prioritized);
        row.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        row.solved = r.solved;
        row.lowLevel = r.expanded;
        if (r.solved) {
            const auto rep = swarmgrid::validatePlan(g, s, t, r.paths);
            if (!rep.ok) throw std::logic_error("invalid prioritized plan");
            row.soc = rep.sumOfCosts;
            row.makespan = rep.makespan;
        }
        return row;
    }
    if (algo == "cbs" || algo == "cbs-m5") {
        swarmgrid::CbsOptions o;
        o.maxNodes = 100000000;
        o.timeLimitSeconds = timeout;
        o.prioritizeConflicts = o.bypass = algo == "cbs-m5";
        const auto r = swarmgrid::solveCbs(g, s, t, o);
        row.solved = r.solved;
        row.timeout = r.limitReached;
        row.seconds = r.seconds;
        row.ctExpanded = r.nodesExpanded;
        row.ctGenerated = r.nodesGenerated;
        row.lowLevel = r.lowLevelExpanded;
        if (r.solved) {
            const auto rep = swarmgrid::validatePlan(g, s, t, r.paths);
            if (!rep.ok) throw std::logic_error("invalid CBS plan");
            row.soc = rep.sumOfCosts;
            row.makespan = rep.makespan;
            row.lowerBound = rep.sumOfCosts;  // optimal
        }
        return row;
    }
    if (algo.rfind("ecbs-", 0) == 0) {
        swarmgrid::EcbsOptions o;
        o.weight = std::stod(algo.substr(5));
        o.maxNodes = 100000000;
        o.timeLimitSeconds = timeout;
        const auto r = swarmgrid::solveEcbs(g, s, t, o);
        row.solved = r.solved;
        row.timeout = r.limitReached;
        row.seconds = r.seconds;
        row.ctExpanded = r.nodesExpanded;
        row.ctGenerated = r.nodesGenerated;
        row.lowLevel = r.lowLevelExpanded;
        if (r.solved) {
            const auto rep = swarmgrid::validatePlan(g, s, t, r.paths);
            if (!rep.ok) throw std::logic_error("invalid ECBS plan");
            row.soc = rep.sumOfCosts;
            row.makespan = rep.makespan;
            row.lowerBound = r.lowerBound;
        }
        return row;
    }
    throw std::invalid_argument("unknown algorithm '" + algo + "' (pp, cbs, cbs-m5, ecbs-<w>)");
}

}  // namespace

int runBench(int argc, char** argv) {
    const Args a(argc, argv, 2, {"out", "map", "scen-dir", "generated", "agents", "scenarios", "timeout", "algos", "shard"});
    if (a.help() || !a.has("out")) {
        std::cout <<
            "swarmgrid bench --out results.csv [options]\n"
            "  --map FILE.map --scen-dir DIR   MovingAI map + a folder with its <map>-*.scen files (other maps' files\n"
            "                                 in the folder are skipped); the first k agents of each file are used\n"
            "  --generated small|large        generated warehouse instead (default: large), random seeds 1..N\n"
            "  --agents 10,20,30,50,100       fleet sizes\n"
            "  --scenarios 25                 instances per fleet size\n"
            "  --timeout 60                   seconds per solve (CBS / ECBS)\n"
            "  --algos pp,cbs,cbs-m5,ecbs-1.05,ecbs-1.2\n"
            "  --shard i/n                    run only jobs i, i+n, i+2n, ... (parallel runs: one CSV each, all go to plot_bench.py)\n";
        return a.help() ? 0 : 1;
    }
    const std::vector<int> agents = a.intList("agents", {10, 20, 30, 50, 100});
    const int scenarios = a.integer("scenarios", 25);
    const double timeout = a.real("timeout", 60.0);
    const std::vector<std::string> algos = a.strList("algos", {"pp", "cbs", "cbs-m5", "ecbs-1.05", "ecbs-1.2"});
    int shardIndex = 0, shardCount = 1;
    if (a.has("shard")) {
        const std::string sh = a.str("shard", "0/1");
        const auto slash = sh.find('/');
        if (slash == std::string::npos) throw std::invalid_argument("--shard: expected i/n");
        shardIndex = std::stoi(sh.substr(0, slash));
        shardCount = std::stoi(sh.substr(slash + 1));
        if (shardCount < 1 || shardIndex < 0 || shardIndex >= shardCount) throw std::invalid_argument("--shard: need 0 <= i < n");
    }
    if (scenarios < 1 || !(timeout > 0)) throw std::invalid_argument("--scenarios >= 1 and --timeout > 0");

    swarmgrid::Grid grid;
    std::string mapName;
    std::vector<std::vector<swarmgrid::ScenarioEntry>> scens;
    if (a.has("map")) {
        if (!a.has("scen-dir")) throw std::invalid_argument("--map needs --scen-dir");
        grid = swarmgrid::loadMovingAIMap(a.str("map", ""));
        mapName = std::filesystem::path(a.str("map", "")).stem().string();
        for (const std::string& f : scenarioFiles(a.str("scen-dir", ""), mapName)) {
            if (static_cast<int>(scens.size()) == scenarios) break;
            scens.push_back(swarmgrid::loadScenario(f));
        }
    } else {
        const std::string which = a.str("generated", "large");
        grid = generatedMap(which);
        mapName = "generated-" + which;
    }

    std::ofstream out(a.str("out", ""), std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + a.str("out", ""));
    // Every shard writes the header, so each file is a valid CSV on its own.
    out << "map,agents,scenario,algorithm,solved,timeout,sum_of_costs,makespan,lower_bound,seconds,"
           "ct_expanded,ct_generated,low_level_expanded\n";
    int job = 0;
    for (int k : agents) {
        for (int sc = 0; sc < scenarios; ++sc) {
            if (job++ % shardCount != shardIndex) continue;
            swarmgrid::MapfInstance inst;
            if (!scens.empty()) {
                if (sc >= static_cast<int>(scens.size())) break;
                inst = swarmgrid::makeInstance(grid, scens[static_cast<std::size_t>(sc)], k);
            } else {
                inst = swarmgrid::generateRandomInstance(grid, k, static_cast<std::uint32_t>(sc + 1));
            }
            const auto s = ids(grid, inst.starts);
            const auto t = ids(grid, inst.goals);
            for (const std::string& algo : algos) {
                const Row r = runAlgorithm(algo, grid, s, t, timeout);
                out << mapName << ',' << k << ',' << sc + 1 << ',' << r.algorithm << ',' << (r.solved ? 1 : 0) << ','
                    << (r.timeout ? 1 : 0) << ',' << r.soc << ',' << r.makespan << ',' << r.lowerBound << ','
                    << std::setprecision(6) << r.seconds << ',' << r.ctExpanded << ',' << r.ctGenerated << ','
                    << r.lowLevel << '\n';
                out.flush();
                std::cerr << mapName << " k=" << k << " scen " << sc + 1 << ' ' << algo << ": "
                          << (r.solved ? "solved" : (r.timeout ? "timeout" : "failed")) << " in " << r.seconds << " s\n";
            }
        }
    }
    return 0;
}

namespace {

swarmgrid::LifelongSolver parseSolver(const std::string& s) {
    if (s == "ecbs") return swarmgrid::LifelongSolver::Ecbs;
    if (s == "cbs") return swarmgrid::LifelongSolver::Cbs;
    if (s == "pp") return swarmgrid::LifelongSolver::Prioritized;
    throw std::invalid_argument("solver: expected ecbs, cbs or pp, got '" + s + "'");
}

swarmgrid::AssignPolicy parseAssign(const std::string& s) {
    if (s == "hungarian") return swarmgrid::AssignPolicy::Hungarian;
    if (s == "greedy") return swarmgrid::AssignPolicy::Greedy;
    throw std::invalid_argument("assign: expected hungarian or greedy, got '" + s + "'");
}

// Options shared by `lifelong` and `lifelong-bench` (single values in both).
swarmgrid::LifelongOptions commonLifelongOptions(const Args& a) {
    swarmgrid::LifelongOptions o;
    o.steps = a.integer("steps", o.steps);
    o.ecbsWeight = a.real("weight", o.ecbsWeight);
    o.breakdownRate = a.real("breakdown-rate", o.breakdownRate);
    o.repairSteps = a.integer("repair", o.repairSteps);
    o.plannerMaxNodes = a.integer("max-nodes", o.plannerMaxNodes);
    return o;
}

swarmgrid::Grid lifelongMap(const Args& a) {
    if (a.has("map")) return swarmgrid::loadMovingAIMap(a.str("map", ""));
    return generatedMap(a.str("generated", "small"));
}

const char* kLifelongHelp =
    "  --map FILE.map | --generated small|large   (default: small)\n"
    "  --robots 20 --steps 500 --seed 1\n"
    "  --window 10 --replan 5        RHCR: resolve conflicts for w steps, replan every h steps (h <= w; w = -1: whole paths)\n"
    "  --solver ecbs|cbs|pp --weight 1.2 --max-nodes 2000\n"
    "  --assign hungarian|greedy\n"
    "  --breakdown-rate 0 --repair 20   chance per robot per step of a breakdown, steps to repair\n";

}  // namespace

int runLifelongCommand(int argc, char** argv) {
    const Args a(argc, argv, 2, {"map", "generated", "robots", "steps", "seed", "window", "replan", "solver", "weight",
                                 "max-nodes", "assign", "breakdown-rate", "repair"});
    if (a.help()) {
        std::cout << "swarmgrid lifelong [options]\n" << kLifelongHelp;
        return 0;
    }
    const swarmgrid::Grid g = lifelongMap(a);
    swarmgrid::LifelongOptions o = commonLifelongOptions(a);
    o.robots = a.integer("robots", o.robots);
    o.window = a.integer("window", o.window);
    o.replanEvery = a.integer("replan", o.replanEvery);
    o.seed = static_cast<std::uint32_t>(a.integer("seed", static_cast<int>(o.seed)));
    o.solver = parseSolver(a.str("solver", "ecbs"));
    o.assignment = parseAssign(a.str("assign", "hungarian"));
    const swarmgrid::LifelongStats s = swarmgrid::runLifelong(g, o);
    std::cout << g.width() << " x " << g.height() << " map, " << o.robots << " robots, " << o.steps << " steps, "
              << swarmgrid::solverName(o.solver) << ", window " << o.window << ", replan every " << o.replanEvery
              << ", " << swarmgrid::assignName(o.assignment) << " assignment\n"
              << "delivered " << s.delivered << " orders, throughput " << s.throughput << " per step\n"
              << "replans " << s.replans << ", planner failures " << s.plannerFailures << ", everyone-waits "
              << s.allWait << "\n"
              << "deadlocks detected " << s.deadlocks << ", escapes " << s.escapes << ", breakdowns "
              << s.breakdowns << ", collisions " << s.collisions << "\n"
              << "planning time per replan: mean " << s.meanPlanSeconds << " s, max " << s.maxPlanSeconds << " s\n";
    return s.collisions == 0 ? 0 : 3;
}

int runLifelongBench(int argc, char** argv) {
    const Args a(argc, argv, 2, {"out", "map", "generated", "robots", "steps", "seeds", "windows", "replan",
                                 "solvers", "assign", "weight", "max-nodes", "breakdown-rate", "repair"});
    if (a.help() || !a.has("out")) {
        std::cout << "swarmgrid lifelong-bench --out lifelong.csv [options]\n"
                     "  --robots 10,20,30 --seeds 5 --steps 500 --windows 5,10,20 --replan 5\n"
                     "  --solvers ecbs --assign greedy,hungarian (each a comma list)\n"
                     "  --map FILE.map | --generated small|large, --weight, --max-nodes, --breakdown-rate, --repair\n";
        return a.help() ? 0 : 1;
    }
    const swarmgrid::Grid g = lifelongMap(a);
    const auto robots = a.intList("robots", {10, 20, 30});
    const int seeds = a.integer("seeds", 5);
    const auto windows = a.intList("windows", {5, 10, 20});
    const int replan = a.integer("replan", 5);
    const auto solvers = a.strList("solvers", {"ecbs"});
    const auto assigns = a.strList("assign", {"greedy", "hungarian"});
    std::ofstream out(a.str("out", ""), std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + a.str("out", ""));
    out << "robots,seed,solver,assign,window,replan,steps,delivered,throughput,mean_plan_s,max_plan_s,"
           "planner_failures,all_wait,deadlocks,escapes,breakdowns,collisions,assigned_distance\n";
    for (int k : robots) {
        for (int w : windows) {
            for (const auto& solver : solvers) {
                for (const auto& assign : assigns) {
                    for (int seed = 1; seed <= seeds; ++seed) {
                        swarmgrid::LifelongOptions o = commonLifelongOptions(a);
                        o.robots = k;
                        o.window = w;
                        o.replanEvery = w < 0 ? replan : std::min(replan, w);
                        o.solver = parseSolver(solver);
                        o.assignment = parseAssign(assign);
                        o.seed = static_cast<std::uint32_t>(seed);
                        const auto s = swarmgrid::runLifelong(g, o);
                        out << k << ',' << seed << ',' << solver << ',' << assign << ',' << w << ',' << o.replanEvery << ','
                            << s.steps << ',' << s.delivered << ',' << s.throughput << ',' << s.meanPlanSeconds << ','
                            << s.maxPlanSeconds << ',' << s.plannerFailures << ',' << s.allWait << ',' << s.deadlocks << ','
                            << s.escapes << ',' << s.breakdowns << ',' << s.collisions << ',' << s.assignedDistance << '\n';
                        out.flush();
                        std::cerr << "k=" << k << " w=" << w << ' ' << solver << ' ' << assign << " seed " << seed << ": "
                                  << s.delivered << " delivered, collisions " << s.collisions << '\n';
                        if (s.collisions != 0) throw std::logic_error("lifelong run had collisions");
                    }
                }
            }
        }
    }
    return 0;
}

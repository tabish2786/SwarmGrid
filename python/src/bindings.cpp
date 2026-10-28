// Python bindings (pybind11) for the SwarmGrid C++ core: the `swarmgrid._core` module.
// The public, documented API is the Python package in python/swarmgrid/__init__.py, which wraps
// these functions in dataclasses and friendlier signatures. Built by `pip install .` (see
// pyproject.toml); not part of the normal CMake build.
//
// Conventions: a point is an (x, y) tuple, x = column, y = row, (0, 0) = top-left. Long searches
// release the GIL, so several solves can run in parallel Python threads.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "swarmgrid/ascii.hpp"
#include "swarmgrid/astar.hpp"
#include "swarmgrid/cbs.hpp"
#include "swarmgrid/conflict_ranking.hpp"
#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/grid.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/lifelong.hpp"
#include "swarmgrid/map_io.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/task_assignment.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace py = pybind11;
using swarmgrid::CellId;
using swarmgrid::Grid;

namespace {

using XY = std::pair<int, int>;
using XYList = std::vector<XY>;

CellId toCell(const Grid& g, const XY& p, const char* what, std::size_t index) {
    if (!g.inBounds(p.first, p.second)) {
        std::ostringstream msg;
        msg << what << ' ' << index << " = (" << p.first << ", " << p.second << ") is outside the "
            << g.width() << " x " << g.height() << " grid";
        throw std::invalid_argument(msg.str());
    }
    return g.toId(p.first, p.second);
}

std::vector<CellId> toCells(const Grid& g, const XYList& points, const char* what) {
    std::vector<CellId> out;
    out.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) out.push_back(toCell(g, points[i], what, i));
    return out;
}

XY toXY(const Grid& g, CellId c) {
    const swarmgrid::Point p = g.toPoint(c);
    return {p.x, p.y};
}

std::vector<XYList> toXYPaths(const Grid& g, const std::vector<swarmgrid::Path>& paths) {
    std::vector<XYList> out;
    out.reserve(paths.size());
    for (const auto& path : paths) {
        XYList p;
        p.reserve(path.size());
        for (CellId c : path) p.push_back(toXY(g, c));
        out.push_back(std::move(p));
    }
    return out;
}

std::vector<swarmgrid::Path> toCellPaths(const Grid& g, const std::vector<XYList>& paths) {
    std::vector<swarmgrid::Path> out;
    for (std::size_t i = 0; i < paths.size(); ++i) out.push_back(toCells(g, paths[i], "path point"));
    return out;
}

XYList toXYList(const std::vector<swarmgrid::Point>& pts) {
    XYList out;
    for (const auto& p : pts) out.emplace_back(p.x, p.y);
    return out;
}

std::pair<int, int> costs(const std::vector<swarmgrid::Path>& paths) {
    int soc = 0, makespan = 0;
    for (const auto& p : paths) {
        soc += static_cast<int>(p.size()) - 1;
        makespan = std::max(makespan, static_cast<int>(p.size()) - 1);
    }
    return {soc, makespan};
}

swarmgrid::ConflictRanker rankerFrom(const std::optional<std::vector<std::int64_t>>& weights) {
    if (!weights) return swarmgrid::ConflictRanker();
    if (static_cast<int>(weights->size()) != swarmgrid::kConflictFeatures) {
        throw std::invalid_argument("ranker weights: expected " + std::to_string(swarmgrid::kConflictFeatures) +
                                    " integers, got " + std::to_string(weights->size()));
    }
    std::array<std::int64_t, swarmgrid::kConflictFeatures> w{};
    for (std::size_t i = 0; i < w.size(); ++i) w[i] = (*weights)[i];
    return swarmgrid::ConflictRanker(w);
}

swarmgrid::ConflictSelection selectionFrom(const std::string& name) {
    if (name == "first") return swarmgrid::ConflictSelection::First;
    if (name == "cardinal") return swarmgrid::ConflictSelection::Cardinal;
    if (name == "learned") return swarmgrid::ConflictSelection::Learned;
    if (name == "oracle") return swarmgrid::ConflictSelection::Oracle;
    throw std::invalid_argument("conflict_selection must be 'first', 'cardinal', 'learned' or 'oracle', got '" +
                                name + "'");
}

py::dict solveCbs(const Grid& g, const XYList& s, const XYList& t, int maxNodes, double timeLimit,
                  const std::string& selection, bool bypass, int window,
                  const std::optional<std::vector<std::int64_t>>& rankerWeights) {
    const auto starts = toCells(g, s, "start");
    const auto goals = toCells(g, t, "goal");
    swarmgrid::CbsOptions o;
    o.maxNodes = maxNodes;
    o.timeLimitSeconds = timeLimit;
    o.selection = selectionFrom(selection);
    o.bypass = bypass;
    o.window = window;
    o.ranker = rankerFrom(rankerWeights);
    swarmgrid::CbsResult r;
    {
        py::gil_scoped_release release;
        r = swarmgrid::solveCbs(g, starts, goals, o);
    }
    py::dict d;
    d["solved"] = r.solved;
    d["limit_reached"] = r.limitReached;
    d["paths"] = toXYPaths(g, r.paths);
    d["sum_of_costs"] = r.sumOfCosts;
    d["makespan"] = costs(r.paths).second;
    d["lower_bound"] = r.solved ? r.sumOfCosts : 0;
    d["nodes_expanded"] = r.nodesExpanded;
    d["nodes_generated"] = r.nodesGenerated;
    d["low_level_expanded"] = r.lowLevelExpanded;
    d["bypasses"] = r.bypasses;
    d["cardinal_splits"] = r.cardinalSplits;
    d["semi_cardinal_splits"] = r.semiCardinalSplits;
    d["non_cardinal_splits"] = r.nonCardinalSplits;
    d["mdds_built"] = r.mddsBuilt;
    d["oracle_replans"] = r.oracleReplans;
    d["seconds"] = r.seconds;
    return d;
}

py::dict solveEcbs(const Grid& g, const XYList& s, const XYList& t, double weight, int maxNodes,
                   double timeLimit, int window) {
    const auto starts = toCells(g, s, "start");
    const auto goals = toCells(g, t, "goal");
    swarmgrid::EcbsOptions o;
    o.weight = weight;
    o.maxNodes = maxNodes;
    o.timeLimitSeconds = timeLimit;
    o.window = window;
    swarmgrid::EcbsResult r;
    {
        py::gil_scoped_release release;
        r = swarmgrid::solveEcbs(g, starts, goals, o);
    }
    py::dict d;
    d["solved"] = r.solved;
    d["limit_reached"] = r.limitReached;
    d["paths"] = toXYPaths(g, r.paths);
    d["sum_of_costs"] = r.sumOfCosts;
    d["makespan"] = costs(r.paths).second;
    d["lower_bound"] = r.lowerBound;
    d["nodes_expanded"] = r.nodesExpanded;
    d["nodes_generated"] = r.nodesGenerated;
    d["low_level_expanded"] = r.lowLevelExpanded;
    d["seconds"] = r.seconds;
    return d;
}

py::dict solvePrioritized(const Grid& g, const XYList& s, const XYList& t, bool independent, int maxAttempts,
                          int window) {
    const auto starts = toCells(g, s, "start");
    const auto goals = toCells(g, t, "goal");
    swarmgrid::MultiAgentResult r;
    double seconds = 0.0;
    {
        py::gil_scoped_release release;
        const auto t0 = std::chrono::steady_clock::now();
        r = swarmgrid::planMultiAgent(g, starts, goals,
                                      independent ? swarmgrid::MultiAgentMode::Independent
                                                  : swarmgrid::MultiAgentMode::Prioritized,
                                      maxAttempts, window);
        seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
    py::dict d;
    d["solved"] = r.solved;
    d["paths"] = toXYPaths(g, r.paths);
    const auto c = costs(r.paths);
    d["sum_of_costs"] = c.first;
    d["makespan"] = c.second;
    d["failed_agent"] = r.failedAgent;
    d["attempts"] = r.attempts;
    d["low_level_expanded"] = r.expanded;
    d["seconds"] = seconds;
    return d;
}

py::dict validate(const Grid& g, const XYList& s, const XYList& t, const std::vector<XYList>& paths) {
    const auto report = swarmgrid::validatePlan(g, toCells(g, s, "start"), toCells(g, t, "goal"), toCellPaths(g, paths));
    py::list conflicts;
    for (const auto& c : report.conflicts) {
        py::dict cd;
        cd["type"] = c.type == swarmgrid::ConflictType::Vertex ? "vertex"
                   : c.type == swarmgrid::ConflictType::Edge ? "edge" : "target";
        cd["agents"] = std::make_pair(c.a1, c.a2);
        cd["cell"] = toXY(g, c.cell);
        cd["to"] = c.type == swarmgrid::ConflictType::Edge ? py::cast(toXY(g, c.to)) : py::none();
        cd["t"] = c.t;
        conflicts.append(cd);
    }
    py::dict d;
    d["ok"] = report.ok;
    d["errors"] = report.errors;
    d["conflicts"] = conflicts;
    d["sum_of_costs"] = report.sumOfCosts;
    d["makespan"] = report.makespan;
    return d;
}

py::dict lifelong(const Grid& g, int robots, int steps, int window, int replanEvery, const std::string& solver,
                  double weight, const std::string& assignment, int backlog, double breakdownRate, int repairSteps,
                  int stuckSteps, int plannerMaxNodes, double plannerTimeLimit, std::uint32_t seed,
                  bool recordPositions) {
    swarmgrid::LifelongOptions o;
    o.robots = robots;
    o.steps = steps;
    o.window = window;
    o.replanEvery = replanEvery;
    if (solver == "ecbs") o.solver = swarmgrid::LifelongSolver::Ecbs;
    else if (solver == "cbs") o.solver = swarmgrid::LifelongSolver::Cbs;
    else if (solver == "prioritized") o.solver = swarmgrid::LifelongSolver::Prioritized;
    else throw std::invalid_argument("solver must be 'ecbs', 'cbs' or 'prioritized', got '" + solver + "'");
    if (assignment == "hungarian") o.assignment = swarmgrid::AssignPolicy::Hungarian;
    else if (assignment == "greedy") o.assignment = swarmgrid::AssignPolicy::Greedy;
    else throw std::invalid_argument("assignment must be 'hungarian' or 'greedy', got '" + assignment + "'");
    o.ecbsWeight = weight;
    o.backlog = backlog;
    o.breakdownRate = breakdownRate;
    o.repairSteps = repairSteps;
    o.stuckSteps = stuckSteps;
    o.plannerMaxNodes = plannerMaxNodes;
    o.plannerTimeLimit = plannerTimeLimit;
    o.seed = seed;
    swarmgrid::LifelongStats st;
    {
        py::gil_scoped_release release;
        st = swarmgrid::runLifelong(g, o, recordPositions);
    }
    py::dict d;
    d["steps"] = st.steps;
    d["delivered"] = st.delivered;
    d["throughput"] = st.throughput;
    d["replans"] = st.replans;
    d["planner_failures"] = st.plannerFailures;
    d["all_wait"] = st.allWait;
    d["deadlocks"] = st.deadlocks;
    d["escapes"] = st.escapes;
    d["breakdowns"] = st.breakdowns;
    d["collisions"] = st.collisions;
    d["assigned_distance"] = st.assignedDistance;
    d["mean_plan_seconds"] = st.meanPlanSeconds;
    d["max_plan_seconds"] = st.maxPlanSeconds;
    std::vector<XYList> positions;
    for (const auto& row : st.positions) {
        XYList p;
        for (CellId c : row) p.push_back(toXY(g, c));
        positions.push_back(std::move(p));
    }
    d["positions"] = positions;
    return d;
}

py::dict conflictSamples(const Grid& g, const XYList& s, const XYList& t, const std::string& collect, int nodes,
                         int conflicts, const std::optional<std::vector<std::int64_t>>& rankerWeights) {
    const auto starts = toCells(g, s, "start");
    const auto goals = toCells(g, t, "goal");
    swarmgrid::CbsOptions o;
    o.selection = selectionFrom(collect);
    o.ranker = rankerFrom(rankerWeights);
    o.sampleMaxNodes = nodes;
    o.sampleMaxConflicts = conflicts;
    o.maxNodes = nodes;
    std::vector<swarmgrid::ConflictSample> samples;
    o.samples = &samples;
    swarmgrid::CbsResult r;
    {
        py::gil_scoped_release release;
        r = swarmgrid::solveCbs(g, starts, goals, o);
    }
    std::vector<std::vector<std::int64_t>> features;
    std::vector<int> node, index, minInc, sumInc;
    for (const auto& smp : samples) {
        features.emplace_back(smp.features.begin(), smp.features.end());
        node.push_back(smp.node);
        index.push_back(smp.conflictIndex);
        minInc.push_back(smp.minIncrease);
        sumInc.push_back(smp.sumIncrease);
    }
    py::dict d;
    d["features"] = features;
    d["node"] = node;
    d["conflict_index"] = index;
    d["min_increase"] = minInc;
    d["sum_increase"] = sumInc;
    d["solved"] = r.solved;
    d["nodes_expanded"] = r.nodesExpanded;
    return d;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "SwarmGrid C++ core (private; use the `swarmgrid` package API).";

    py::class_<Grid>(m, "Grid", "A 4-connected grid of free cells and obstacles. (x, y): x = column, y = row.")
        .def(py::init<int, int>(), py::arg("width"), py::arg("height"),
             "An all-free grid. Raises ValueError if a size is not positive.")
        .def_property_readonly("width", &Grid::width)
        .def_property_readonly("height", &Grid::height)
        .def("in_bounds", &Grid::inBounds, py::arg("x"), py::arg("y"))
        .def("is_free", py::overload_cast<int, int>(&Grid::isFree, py::const_), py::arg("x"), py::arg("y"),
             "True if (x, y) is inside the grid and not an obstacle.")
        .def("set_obstacle", [](Grid& g, int x, int y, bool blocked) {
                 if (!g.inBounds(x, y)) throw std::invalid_argument("set_obstacle: (x, y) is outside the grid");
                 g.setObstacle(x, y, blocked);
             },
             py::arg("x"), py::arg("y"), py::arg("blocked") = true)
        .def("count_obstacles", &Grid::countObstacles)
        .def("rows", [](const Grid& g) {
                 std::vector<std::string> rows;
                 for (int y = 0; y < g.height(); ++y) {
                     std::string row;
                     for (int x = 0; x < g.width(); ++x) row += g.isFree(x, y) ? '.' : '@';
                     rows.push_back(row);
                 }
                 return rows;
             },
             "The map as strings, '.' = free, '@' = obstacle (MovingAI style).")
        .def("free_cells", [](const Grid& g) {
                 XYList out;
                 for (int y = 0; y < g.height(); ++y) {
                     for (int x = 0; x < g.width(); ++x) {
                         if (g.isFree(x, y)) out.emplace_back(x, y);
                     }
                 }
                 return out;
             })
        .def("copy", [](const Grid& g) { return Grid(g); })
        .def("__copy__", [](const Grid& g) { return Grid(g); })
        .def("__deepcopy__", [](const Grid& g, py::dict) { return Grid(g); }, py::arg("memo"))
        .def("__eq__", [](const Grid& a, const Grid& b) {
            if (a.width() != b.width() || a.height() != b.height()) return false;
            for (CellId c = 0; c < a.numCells(); ++c) {
                if (a.isFree(c) != b.isFree(c)) return false;
            }
            return true;
        })
        .def("__repr__", [](const Grid& g) {
            return "Grid(" + std::to_string(g.width()) + "x" + std::to_string(g.height()) + ", " +
                   std::to_string(g.countObstacles()) + " obstacles)";
        });

    m.def("grid_from_rows", [](const std::vector<std::string>& rows) {
              if (rows.empty()) throw std::invalid_argument("grid_from_rows: no rows");
              std::ostringstream text;
              text << "type octile\nheight " << rows.size() << "\nwidth " << rows[0].size() << "\nmap\n";
              for (const auto& r : rows) text << r << '\n';
              std::istringstream in(text.str());
              try {
                  return swarmgrid::parseMovingAIMap(in);
              } catch (const std::runtime_error& e) {  // bad rows are bad input here: ValueError
                  throw std::invalid_argument(std::string("grid_from_rows: ") + e.what());
              }
          },
          py::arg("rows"));
    m.def("warehouse", [](int bx, int by, int sw, int sh, int aisle, int border) {
              swarmgrid::WarehouseParams p;
              p.blocksX = bx;
              p.blocksY = by;
              p.shelfBlockWidth = sw;
              p.shelfBlockHeight = sh;
              p.aisleWidth = aisle;
              p.border = border;
              return swarmgrid::generateWarehouse(p);
          },
          py::arg("blocks_x"), py::arg("blocks_y"), py::arg("shelf_width"), py::arg("shelf_height"),
          py::arg("aisle"), py::arg("border"));
    m.def("load_map", &swarmgrid::loadMovingAIMap, py::arg("path"));
    m.def("scenario_agents", [](const Grid& g, const std::string& path, int agents) {
              const auto inst = swarmgrid::makeInstance(g, swarmgrid::loadScenario(path), agents);
              return std::make_pair(toXYList(inst.starts), toXYList(inst.goals));
          },
          py::arg("grid"), py::arg("path"), py::arg("agents"));
    m.def("random_agents", [](const Grid& g, int n, std::uint32_t seed) {
              const auto inst = swarmgrid::generateRandomInstance(g, n, seed);
              return std::make_pair(toXYList(inst.starts), toXYList(inst.goals));
          },
          py::arg("grid"), py::arg("agents"), py::arg("seed"));
    m.def("shortest_path", [](const Grid& g, const XY& start, const XY& goal) -> std::optional<XYList> {
              const CellId s = toCell(g, start, "start", 0);
              const CellId t = toCell(g, goal, "goal", 0);
              if (!g.isFree(s) || !g.isFree(t)) throw std::invalid_argument("start and goal must be free cells");
              const auto path = swarmgrid::astar(g, s, t, swarmgrid::bfsDistances(g, t));
              if (!path) return std::nullopt;
              XYList out;
              for (CellId c : *path) out.push_back(toXY(g, c));
              return out;
          },
          py::arg("grid"), py::arg("start"), py::arg("goal"));
    m.def("render", [](const Grid& g, const XYList& positions) {
              std::vector<swarmgrid::Point> pts;
              for (std::size_t i = 0; i < positions.size(); ++i) {
                  toCell(g, positions[i], "position", i);
                  pts.push_back({positions[i].first, positions[i].second});
              }
              return pts.empty() ? swarmgrid::renderGrid(g) : swarmgrid::renderAgents(g, pts);
          },
          py::arg("grid"), py::arg("positions"));

    m.def("solve_cbs", &solveCbs, py::arg("grid"), py::arg("starts"), py::arg("goals"), py::arg("max_nodes"),
          py::arg("time_limit"), py::arg("conflict_selection"), py::arg("bypass"), py::arg("window"),
          py::arg("ranker_weights"));
    m.def("solve_ecbs", &solveEcbs, py::arg("grid"), py::arg("starts"), py::arg("goals"), py::arg("weight"),
          py::arg("max_nodes"), py::arg("time_limit"), py::arg("window"));
    m.def("solve_prioritized", &solvePrioritized, py::arg("grid"), py::arg("starts"), py::arg("goals"),
          py::arg("independent"), py::arg("max_attempts"), py::arg("window"));
    m.def("validate", &validate, py::arg("grid"), py::arg("starts"), py::arg("goals"), py::arg("paths"));
    m.def("assign", [](const Grid& g, const XYList& robots, const XYList& orders, bool optimal) {
              const auto r = toCells(g, robots, "robot");
              const auto o = toCells(g, orders, "order");
              const auto pairs = optimal ? swarmgrid::assignOptimal(g, r, o) : swarmgrid::assignNearest(g, r, o);
              std::vector<std::tuple<int, int, int>> out;
              for (const auto& a : pairs) out.emplace_back(a.robot, a.order, a.distance);
              return out;
          },
          py::arg("grid"), py::arg("robots"), py::arg("orders"), py::arg("optimal"));
    m.def("lifelong", &lifelong, py::arg("grid"), py::arg("robots"), py::arg("steps"), py::arg("window"),
          py::arg("replan_every"), py::arg("solver"), py::arg("weight"), py::arg("assignment"), py::arg("backlog"),
          py::arg("breakdown_rate"), py::arg("repair_steps"), py::arg("stuck_steps"), py::arg("planner_max_nodes"),
          py::arg("planner_time_limit"), py::arg("seed"), py::arg("record_positions"));
    m.def("conflict_samples", &conflictSamples, py::arg("grid"), py::arg("starts"), py::arg("goals"),
          py::arg("collect"), py::arg("nodes"), py::arg("conflicts"), py::arg("ranker_weights"));
    m.def("conflict_feature_names", [] {
        std::vector<std::string> names;
        for (int i = 0; i < swarmgrid::kConflictFeatures; ++i) names.emplace_back(swarmgrid::conflictFeatureName(i));
        return names;
    });
    // Keep the ranker in a named variable: weights() returns a reference into it, so binding that
    // reference to a temporary ranker would dangle.
    m.def("builtin_ranker_weights", [] {
        const swarmgrid::ConflictRanker ranker;
        return std::vector<std::int64_t>(ranker.weights().begin(), ranker.weights().end());
    });
    m.def("load_ranker_weights", [](const std::string& path) {
              const swarmgrid::ConflictRanker ranker = swarmgrid::ConflictRanker::load(path);
              return std::vector<std::int64_t>(ranker.weights().begin(), ranker.weights().end());
          },
          py::arg("path"));
}

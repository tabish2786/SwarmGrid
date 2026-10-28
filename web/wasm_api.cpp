// C interface to the planner for the browser (docs/control.html).
// Compiled to WebAssembly with Emscripten by web/build_wasm.sh; not part of the native build.
//
// Usage from JavaScript: add agents with sg_add_agent (or sg_random_agents), call sg_plan, then
// read the paths back with sg_path_length / sg_path_cell. Only plain ints cross the boundary,
// so no memory has to be shared or freed on the JavaScript side.

#include <emscripten/emscripten.h>

#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/task_assignment.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace {

swarmgrid::Grid g_grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
std::vector<swarmgrid::CellId> g_starts;
std::vector<swarmgrid::CellId> g_goals;
swarmgrid::MultiAgentResult g_result;
int g_ctNodes = 0;
int g_limitReached = 0;
double g_ecbsWeight = 1.2;
double g_lowerBound = 0;
std::string g_error;
std::vector<swarmgrid::CellId> g_assignRobots;
std::vector<swarmgrid::CellId> g_assignOrders;
std::vector<swarmgrid::Assignment> g_assignments;

bool validAgent(int i) { return i >= 0 && i < static_cast<int>(g_starts.size()); }

}  // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE int sg_width() { return g_grid.width(); }
EMSCRIPTEN_KEEPALIVE int sg_height() { return g_grid.height(); }

EMSCRIPTEN_KEEPALIVE int sg_is_free(int cell) {
    return cell >= 0 && cell < g_grid.numCells() && g_grid.isFree(cell) ? 1 : 0;
}

// Map editing (shelves, broken robots). Returns 1, or 0 for a cell outside the grid.
EMSCRIPTEN_KEEPALIVE int sg_set_obstacle(int cell, int blocked) {
    if (cell < 0 || cell >= g_grid.numCells()) return 0;
    const swarmgrid::Point p = g_grid.toPoint(cell);
    g_grid.setObstacle(p.x, p.y, blocked != 0);
    return 1;
}

EMSCRIPTEN_KEEPALIVE void sg_reset_map() {
    g_grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
}

// Order assignment (see task_assignment.hpp): add the idle robots' cells and the open orders'
// cells, run (greedy nearest pair, or optimal Hungarian), then read the pairs back. Indices are
// in the order added.
EMSCRIPTEN_KEEPALIVE void sg_assign_clear() {
    g_assignRobots.clear();
    g_assignOrders.clear();
    g_assignments.clear();
}
EMSCRIPTEN_KEEPALIVE void sg_assign_add_robot(int cell) { g_assignRobots.push_back(cell); }
EMSCRIPTEN_KEEPALIVE void sg_assign_add_order(int cell) { g_assignOrders.push_back(cell); }

// Returns the number of pairs, or -1 for invalid input (see sg_last_error).
// optimal = 0: greedy nearest pair; 1: Hungarian (most pairs, then least total distance).
EMSCRIPTEN_KEEPALIVE int sg_assign_run_policy(int optimal) {
    try {
        g_assignments = optimal ? swarmgrid::assignOptimal(g_grid, g_assignRobots, g_assignOrders)
                                : swarmgrid::assignNearest(g_grid, g_assignRobots, g_assignOrders);
        return static_cast<int>(g_assignments.size());
    } catch (const std::exception& e) {
        g_assignments.clear();
        g_error = e.what();
        return -1;
    }
}
EMSCRIPTEN_KEEPALIVE int sg_assign_run() { return sg_assign_run_policy(0); }
EMSCRIPTEN_KEEPALIVE int sg_assign_distance(int i) {
    return i >= 0 && i < static_cast<int>(g_assignments.size()) ? g_assignments[static_cast<std::size_t>(i)].distance : -1;
}
EMSCRIPTEN_KEEPALIVE int sg_assign_robot(int i) {
    return i >= 0 && i < static_cast<int>(g_assignments.size()) ? g_assignments[static_cast<std::size_t>(i)].robot : -1;
}
EMSCRIPTEN_KEEPALIVE int sg_assign_order(int i) {
    return i >= 0 && i < static_cast<int>(g_assignments.size()) ? g_assignments[static_cast<std::size_t>(i)].order : -1;
}

EMSCRIPTEN_KEEPALIVE void sg_clear_agents() {
    g_starts.clear();
    g_goals.clear();
    g_result = swarmgrid::MultiAgentResult{};
}

EMSCRIPTEN_KEEPALIVE void sg_add_agent(int start, int goal) {
    g_starts.push_back(start);
    g_goals.push_back(goal);
}

// Replaces the agents with n random ones (distinct starts, distinct goals), using the same
// portable RNG as the native CLI, so a seed gives the same scene everywhere. Returns n, or -1.
EMSCRIPTEN_KEEPALIVE int sg_random_agents(int n, std::uint32_t seed) {
    try {
        const swarmgrid::MapfInstance inst = swarmgrid::generateRandomInstance(g_grid, n, seed);
        sg_clear_agents();
        for (int i = 0; i < n; ++i) {
            sg_add_agent(g_grid.toId(inst.starts[static_cast<std::size_t>(i)]),
                         g_grid.toId(inst.goals[static_cast<std::size_t>(i)]));
        }
        return n;
    } catch (const std::exception& e) {
        g_error = e.what();
        return -1;
    }
}

EMSCRIPTEN_KEEPALIVE int sg_agent_count() { return static_cast<int>(g_starts.size()); }
EMSCRIPTEN_KEEPALIVE int sg_agent_start(int i) { return validAgent(i) ? g_starts[static_cast<std::size_t>(i)] : -1; }
EMSCRIPTEN_KEEPALIVE int sg_agent_goal(int i) { return validAgent(i) ? g_goals[static_cast<std::size_t>(i)] : -1; }

// Suboptimality bound for mode 3 (ECBS). Values below 1 are raised to 1.
EMSCRIPTEN_KEEPALIVE void sg_set_ecbs_weight(double w) { g_ecbsWeight = w < 1.0 ? 1.0 : w; }

// mode 0 = independent A*, 1 = prioritized planning, 2 = CBS (optimal, with the M5
// improvements; stops after maxNodes constraint-tree nodes or timeLimitSeconds, see
// sg_limit_reached), 3 = ECBS (sum of costs <= weight * optimal, same limits; see
// sg_lower_bound), 4 = CBS with the learned conflict ranker (conflict_ranking.hpp; also optimal).
// Returns 1 if every agent got a path, 0 if planning failed (see sg_failed_agent /
// sg_limit_reached), -1 if the input was invalid (see sg_last_error).
EMSCRIPTEN_KEEPALIVE int sg_plan_with_limits(int mode, int maxNodes, double timeLimitSeconds) {
    try {
        g_error.clear();
        g_ctNodes = 0;
        g_limitReached = 0;
        g_lowerBound = 0;
        if (mode == 3) {
            swarmgrid::EcbsOptions opts;
            opts.weight = g_ecbsWeight;
            opts.maxNodes = maxNodes;
            opts.timeLimitSeconds = timeLimitSeconds;
            const swarmgrid::EcbsResult r = swarmgrid::solveEcbs(g_grid, g_starts, g_goals, opts);
            g_result = swarmgrid::MultiAgentResult{};
            g_result.solved = r.solved;
            g_result.paths = r.paths;
            g_result.attempts = 1;
            g_result.expanded = r.lowLevelExpanded;
            g_ctNodes = r.nodesExpanded;
            g_limitReached = r.limitReached ? 1 : 0;
            g_lowerBound = static_cast<double>(r.lowerBound);
            return r.solved ? 1 : 0;
        }
        if (mode == 2 || mode == 4) {
            swarmgrid::CbsOptions opts;
            opts.maxNodes = maxNodes;
            opts.timeLimitSeconds = timeLimitSeconds;
            if (mode == 4) opts.selection = swarmgrid::ConflictSelection::Learned;
            const swarmgrid::CbsResult r = swarmgrid::solveCbs(g_grid, g_starts, g_goals, opts);
            g_result = swarmgrid::MultiAgentResult{};
            g_result.solved = r.solved;
            g_result.paths = r.paths;
            g_result.attempts = 1;
            g_result.expanded = r.lowLevelExpanded;
            g_ctNodes = r.nodesExpanded;
            g_limitReached = r.limitReached ? 1 : 0;
            g_lowerBound = static_cast<double>(r.sumOfCosts);
            return r.solved ? 1 : 0;
        }
        g_result = swarmgrid::planMultiAgent(
            g_grid, g_starts, g_goals,
            mode == 0 ? swarmgrid::MultiAgentMode::Independent : swarmgrid::MultiAgentMode::Prioritized);
        return g_result.solved ? 1 : 0;
    } catch (const std::exception& e) {
        g_result = swarmgrid::MultiAgentResult{};
        g_error = e.what();
        return -1;
    }
}

EMSCRIPTEN_KEEPALIVE int sg_plan(int mode) { return sg_plan_with_limits(mode, 20000, 3.0); }

EMSCRIPTEN_KEEPALIVE int sg_path_length(int i) {
    return i >= 0 && i < static_cast<int>(g_result.paths.size())
               ? static_cast<int>(g_result.paths[static_cast<std::size_t>(i)].size())
               : 0;
}

EMSCRIPTEN_KEEPALIVE int sg_path_cell(int i, int t) {
    if (t < 0 || t >= sg_path_length(i)) return -1;
    return g_result.paths[static_cast<std::size_t>(i)][static_cast<std::size_t>(t)];
}

EMSCRIPTEN_KEEPALIVE int sg_failed_agent() { return g_result.failedAgent; }
EMSCRIPTEN_KEEPALIVE int sg_ct_nodes() { return g_ctNodes; }
EMSCRIPTEN_KEEPALIVE int sg_limit_reached() { return g_limitReached; }
// After a solved CBS (mode 2) or ECBS (mode 3): no plan has a smaller sum of costs than this.
EMSCRIPTEN_KEEPALIVE double sg_lower_bound() { return g_lowerBound; }
EMSCRIPTEN_KEEPALIVE int sg_attempts() { return g_result.attempts; }
EMSCRIPTEN_KEEPALIVE double sg_expanded() { return static_cast<double>(g_result.expanded); }
EMSCRIPTEN_KEEPALIVE const char* sg_last_error() { return g_error.c_str(); }

}  // extern "C"

#include "swarmgrid/prioritized.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "swarmgrid/constraints.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/space_time_astar.hpp"

namespace swarmgrid {

namespace {

void validate(const Grid& grid, const std::vector<CellId>& starts,
              const std::vector<CellId>& goals, int maxAttempts) {
    if (starts.empty() || starts.size() != goals.size()) {
        throw std::invalid_argument("planMultiAgent: need the same, non-zero number of starts and goals");
    }
    if (maxAttempts < 1) {
        throw std::invalid_argument("planMultiAgent: maxAttempts must be >= 1");
    }
    std::unordered_set<CellId> seenStarts;
    std::unordered_set<CellId> seenGoals;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        for (CellId c : {starts[i], goals[i]}) {
            if (c < 0 || c >= grid.numCells() || !grid.isFree(c)) {
                throw std::invalid_argument("planMultiAgent: agent " + std::to_string(i) +
                                            " has a start or goal that is not a free cell");
            }
        }
        if (!seenStarts.insert(starts[i]).second || !seenGoals.insert(goals[i]).second) {
            throw std::invalid_argument("planMultiAgent: agent " + std::to_string(i) +
                                        " shares its start or goal with another agent");
        }
    }
}

// Turns a planned path into the constraints every later agent must respect.
void reserve(ConstraintTable& table, const Path& path) {
    for (std::size_t t = 0; t < path.size(); ++t) {
        const int time = static_cast<int>(t);
        table.add(vertexConstraint(0, path[t], time));
        if (t > 0 && path[t] != path[t - 1]) {
            // This agent moved path[t-1] -> path[t]; nobody may move the other way at once.
            table.add(edgeConstraint(0, path[t], path[t - 1], time));
        }
    }
    // It parks on its goal and never leaves.
    table.add(blockedFromConstraint(0, path.back(), static_cast<int>(path.size()) - 1));
}

}  // namespace

MultiAgentResult planMultiAgent(const Grid& grid, const std::vector<CellId>& starts,
                                const std::vector<CellId>& goals, MultiAgentMode mode,
                                int maxAttempts) {
    validate(grid, starts, goals, maxAttempts);
    const std::size_t n = starts.size();

    // One BFS table per goal, reused by every attempt: this is where precomputing pays off.
    std::vector<std::vector<int>> h(n);
    for (std::size_t i = 0; i < n; ++i) h[i] = bfsDistances(grid, goals[i]);

    MultiAgentResult result;
    if (mode == MultiAgentMode::Independent) {
        result.attempts = 1;
        for (std::size_t i = 0; i < n; ++i) {
            SearchStats stats;
            auto path = astar(grid, starts[i], goals[i], h[i], &stats);
            result.expanded += stats.expanded;
            if (!path) {
                result.failedAgent = static_cast<int>(i);
                result.paths.clear();
                return result;
            }
            result.paths.push_back(std::move(*path));
        }
        result.solved = true;
        return result;
    }

    std::vector<int> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = static_cast<int>(i);

    for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
        result.attempts = attempt;
        result.order = order;
        ConstraintTable reserved;  // everything the agents planned so far occupy
        std::vector<Path> paths(n);
        int failed = -1;
        for (int agent : order) {
            const auto a = static_cast<std::size_t>(agent);
            SearchStats stats;
            auto path = spaceTimeAstar(grid, starts[a], goals[a], h[a], reserved, -1, &stats);
            result.expanded += stats.expanded;
            if (!path) {
                failed = agent;
                break;
            }
            reserve(reserved, *path);
            paths[a] = std::move(*path);
        }
        if (failed < 0) {
            result.solved = true;
            result.failedAgent = -1;
            result.paths = std::move(paths);
            return result;
        }
        result.failedAgent = failed;
        if (failed == order.front()) {
            break;  // it failed with no other agent in its way: no order can help
        }
        // Give the agent that got stuck the highest priority and try again.
        order.erase(std::find(order.begin(), order.end(), failed));
        order.insert(order.begin(), failed);
    }
    return result;
}

}  // namespace swarmgrid

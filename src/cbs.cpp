#include "swarmgrid/cbs.hpp"

#include <chrono>
#include <cstddef>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>

#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/constraints.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/space_time_astar.hpp"

namespace swarmgrid {

namespace {

struct CtNode {
    std::vector<Constraint> constraints;  // everything added on the way down from the root
    std::vector<Path> paths;
    long long cost = 0;                   // sum of costs
    int conflicts = 0;                    // number of conflicts among `paths` (tie-break)
};

long long sumOfCosts(const std::vector<Path>& paths) {
    long long s = 0;
    for (const Path& p : paths) s += static_cast<long long>(p.size()) - 1;
    return s;
}

}  // namespace

CbsResult solveCbs(const Grid& grid, const std::vector<CellId>& starts,
                   const std::vector<CellId>& goals, const CbsOptions& options) {
    requireValidAgents(grid, starts, goals, "solveCbs");
    if (options.maxNodes < 1) throw std::invalid_argument("solveCbs: maxNodes must be >= 1");

    using Clock = std::chrono::steady_clock;
    const auto startTime = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - startTime).count(); };

    const std::size_t n = starts.size();
    CbsResult result;

    // One BFS table per goal, reused by every replan of that agent in the whole tree.
    std::vector<std::vector<int>> h(n);
    for (std::size_t i = 0; i < n; ++i) h[i] = bfsDistances(grid, goals[i]);

    auto replan = [&](const std::vector<Constraint>& constraints, int agent) {
        const auto a = static_cast<std::size_t>(agent);
        SearchStats stats;
        auto path = spaceTimeAstar(grid, starts[a], goals[a], h[a], ConstraintTable(constraints, agent),
                                   -1, &stats);
        result.lowLevelExpanded += stats.expanded;
        return path;
    };

    // Nodes live in a vector and are referred to by index: indices stay valid when the vector
    // grows, pointers into it would not.
    std::vector<CtNode> nodes;
    auto worse = [&nodes](int a, int b) {  // true if a should pop after b
        const CtNode& x = nodes[static_cast<std::size_t>(a)];
        const CtNode& y = nodes[static_cast<std::size_t>(b)];
        if (x.cost != y.cost) return x.cost > y.cost;
        if (x.conflicts != y.conflicts) return x.conflicts > y.conflicts;
        return a > b;  // older node first
    };
    std::priority_queue<int, std::vector<int>, decltype(worse)> open(worse);

    CtNode root;
    for (std::size_t i = 0; i < n; ++i) {
        auto path = replan(root.constraints, static_cast<int>(i));
        if (!path) {  // cannot reach its goal even alone: unsolvable
            result.seconds = elapsed();
            return result;
        }
        root.paths.push_back(std::move(*path));
    }
    root.cost = sumOfCosts(root.paths);
    root.conflicts = static_cast<int>(findAllConflicts(root.paths).size());
    nodes.push_back(std::move(root));
    open.push(0);
    result.nodesGenerated = 1;

    while (!open.empty()) {
        if (result.nodesExpanded >= options.maxNodes ||
            (options.timeLimitSeconds > 0 && elapsed() > options.timeLimitSeconds)) {
            result.limitReached = true;
            break;
        }
        const int index = open.top();
        open.pop();
        ++result.nodesExpanded;

        const std::optional<Conflict> conflict = findFirstConflict(nodes[static_cast<std::size_t>(index)].paths);
        if (!conflict) {
            CtNode& best = nodes[static_cast<std::size_t>(index)];
            result.solved = true;
            result.paths = best.paths;
            result.sumOfCosts = best.cost;
            break;
        }

        // The two ways out of this conflict, one per agent.
        std::pair<int, Constraint> branches[2];
        if (conflict->type == ConflictType::Edge) {
            branches[0] = {conflict->a1, edgeConstraint(conflict->a1, conflict->cell, conflict->to, conflict->t)};
            branches[1] = {conflict->a2, edgeConstraint(conflict->a2, conflict->to, conflict->cell, conflict->t)};
        } else {
            branches[0] = {conflict->a1, vertexConstraint(conflict->a1, conflict->cell, conflict->t)};
            branches[1] = {conflict->a2, vertexConstraint(conflict->a2, conflict->cell, conflict->t)};
        }

        for (const auto& [agent, constraint] : branches) {
            CtNode child;
            child.constraints = nodes[static_cast<std::size_t>(index)].constraints;
            child.constraints.push_back(constraint);
            auto path = replan(child.constraints, agent);
            if (!path) continue;  // this branch has no solution
            child.paths = nodes[static_cast<std::size_t>(index)].paths;
            child.paths[static_cast<std::size_t>(agent)] = std::move(*path);
            child.cost = sumOfCosts(child.paths);
            child.conflicts = static_cast<int>(findAllConflicts(child.paths).size());
            nodes.push_back(std::move(child));  // may reallocate: no references held across this
            open.push(static_cast<int>(nodes.size()) - 1);
            ++result.nodesGenerated;
        }
    }
    result.seconds = elapsed();
    return result;
}

}  // namespace swarmgrid

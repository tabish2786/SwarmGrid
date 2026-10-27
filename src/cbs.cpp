#include "swarmgrid/cbs.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>

#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/constraints.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/mdd.hpp"
#include "swarmgrid/space_time_astar.hpp"

namespace swarmgrid {

namespace {

struct CtNode {
    std::vector<Constraint> constraints;  // everything added on the way down from the root
    std::vector<Path> paths;
    long long cost = 0;                   // sum of costs
    int conflicts = 0;                    // number of conflicts among `paths` (tie-break)
    // M5: MDD of each agent under this node's constraints, built on demand. A child shares the
    // parent's MDDs except for the one agent it replanned (whose constraints changed).
    std::vector<std::shared_ptr<const Mdd>> mdds;
};

long long sumOfCosts(const std::vector<Path>& paths) {
    long long s = 0;
    for (const Path& p : paths) s += static_cast<long long>(p.size()) - 1;
    return s;
}

// Does every optimal path of this agent stand on `cell` at time t? After its arrival time the
// agent is parked on its goal, so levels past the end read as the last (goal) level.
bool singletonAt(const Mdd& mdd, int t, CellId cell) {
    if (mdd.levels.empty()) return false;
    const auto i = std::min(static_cast<std::size_t>(t), mdd.levels.size() - 1);
    return mdd.levels[i].size() == 1 && mdd.levels[i][0] == cell;
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
    root.conflicts = static_cast<int>(findAllConflicts(root.paths, options.window).size());
    root.mdds.resize(n);
    nodes.push_back(std::move(root));
    open.push(0);
    result.nodesGenerated = 1;

    auto mddOf = [&](CtNode& node, int agent) -> const Mdd& {
        auto& slot = node.mdds[static_cast<std::size_t>(agent)];
        if (!slot) {
            const auto a = static_cast<std::size_t>(agent);
            slot = std::make_shared<const Mdd>(buildMdd(grid, starts[a], goals[a],
                                                        static_cast<int>(node.paths[a].size()) - 1,
                                                        h[a], ConstraintTable(node.constraints, agent)));
            ++result.mddsBuilt;
        }
        return *slot;
    };

    // How many of the two agents are forced to a longer path by their branch (0, 1 or 2).
    auto cardinality = [&](CtNode& node, const Conflict& c) {
        int forced = 0;
        if (c.type == ConflictType::Edge) {
            const Mdd& m1 = mddOf(node, c.a1);
            if (singletonAt(m1, c.t - 1, c.cell) && singletonAt(m1, c.t, c.to)) ++forced;
            const Mdd& m2 = mddOf(node, c.a2);
            if (singletonAt(m2, c.t - 1, c.to) && singletonAt(m2, c.t, c.cell)) ++forced;
        } else {
            if (singletonAt(mddOf(node, c.a1), c.t, c.cell)) ++forced;
            if (singletonAt(mddOf(node, c.a2), c.t, c.cell)) ++forced;
        }
        return forced;
    };

    while (!open.empty()) {
        if (result.nodesExpanded >= options.maxNodes ||
            (options.timeLimitSeconds > 0 && elapsed() > options.timeLimitSeconds)) {
            result.limitReached = true;
            break;
        }
        const int index = open.top();
        open.pop();
        ++result.nodesExpanded;
        const auto parentIndex = static_cast<std::size_t>(index);

        std::optional<Conflict> conflict;
        if (options.prioritizeConflicts) {
            const std::vector<Conflict> all = findAllConflicts(nodes[parentIndex].paths, options.window);
            int bestForced = -1;
            for (const Conflict& c : all) {  // first cardinal, else first semi-cardinal, else first
                const int forced = cardinality(nodes[parentIndex], c);
                if (forced > bestForced) {
                    bestForced = forced;
                    conflict = c;
                    if (forced == 2) break;
                }
            }
            if (conflict) {
                if (bestForced == 2) ++result.cardinalSplits;
                else if (bestForced == 1) ++result.semiCardinalSplits;
                else ++result.nonCardinalSplits;
            }
        } else {
            conflict = findFirstConflict(nodes[parentIndex].paths, options.window);
            if (conflict) ++result.nonCardinalSplits;
        }
        if (!conflict) {
            CtNode& best = nodes[parentIndex];
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

        std::vector<std::pair<int, CtNode>> children;  // (replanned agent, node)
        for (const auto& [agent, constraint] : branches) {
            const CtNode& parent = nodes[parentIndex];
            CtNode child;
            child.constraints = parent.constraints;
            child.constraints.push_back(constraint);
            auto path = replan(child.constraints, agent);
            if (!path) continue;  // this branch has no solution
            child.paths = parent.paths;
            child.paths[static_cast<std::size_t>(agent)] = std::move(*path);
            child.cost = sumOfCosts(child.paths);
            child.conflicts = static_cast<int>(findAllConflicts(child.paths, options.window).size());
            child.mdds = parent.mdds;
            child.mdds[static_cast<std::size_t>(agent)].reset();  // its constraints changed
            children.emplace_back(agent, std::move(child));
        }

        if (options.bypass) {
            // A child just as cheap with fewer conflicts: its new path also satisfies the parent's
            // constraints (a subset) and has the same length, so adopt it and look again.
            CtNode& parent = nodes[parentIndex];
            auto it = std::find_if(children.begin(), children.end(), [&](const auto& ch) {
                return ch.second.cost == parent.cost && ch.second.conflicts < parent.conflicts;
            });
            if (it != children.end()) {
                const auto a = static_cast<std::size_t>(it->first);
                parent.paths[a] = std::move(it->second.paths[a]);
                parent.conflicts = it->second.conflicts;
                // Same constraints and same cost: the agent's MDD is unchanged, keep it.
                ++result.bypasses;
                open.push(index);
                continue;
            }
        }
        for (auto& child : children) {
            nodes.push_back(std::move(child.second));  // may reallocate: no references held across this
            open.push(static_cast<int>(nodes.size()) - 1);
            ++result.nodesGenerated;
        }
    }
    result.seconds = elapsed();
    return result;
}

}  // namespace swarmgrid

#include "swarmgrid/cbs.hpp"

#include <algorithm>
#include <array>
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

// A CT node stores only what differs from its parent. Trees reach millions of nodes, and copying
// every path and constraint into each node cost about 3.6 KB per node: a 60-second run on the
// small warehouse grew past 5 GB and was killed.
struct CtNode {
    int parent = -1;       // index of the parent node; -1 for the root
    Constraint constraint; // the constraint added here (unused at the root)
    // Paths that differ from the parent's: the replanned agent's, plus any adopted later by bypass
    // (at most one entry per agent). The root lists every agent.
    std::vector<std::pair<int, Path>> paths;
    long long cost = 0;    // sum of costs
    int conflicts = 0;     // number of conflicts among the node's paths (tie-break)
    // M5: MDDs cached at the node where the agent's constraints last changed (the root for an
    // agent never constrained). Every descendant that does not constrain the agent again shares it.
    std::vector<std::pair<int, std::shared_ptr<const Mdd>>> mdds;
};

// Full state of the node being expanded, rebuilt from the chain of its ancestors.
struct NodeState {
    int index = 0;
    std::vector<Path> paths;              // every agent's path
    std::vector<Constraint> constraints;  // everything added from the root down to the node
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
    if (options.samples != nullptr && (options.sampleMaxNodes < 1 || options.sampleMaxConflicts < 1)) {
        throw std::invalid_argument("solveCbs: sampleMaxNodes and sampleMaxConflicts must be >= 1");
    }

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

    {
        CtNode root;
        std::vector<Path> rootPaths;
        for (std::size_t i = 0; i < n; ++i) {
            auto path = replan({}, static_cast<int>(i));
            if (!path) {  // cannot reach its goal even alone: unsolvable
                result.seconds = elapsed();
                return result;
            }
            rootPaths.push_back(*path);
            root.paths.emplace_back(static_cast<int>(i), std::move(*path));
        }
        root.cost = sumOfCosts(rootPaths);
        root.conflicts = static_cast<int>(findAllConflicts(rootPaths, options.window).size());
        nodes.push_back(std::move(root));
    }
    open.push(0);
    result.nodesGenerated = 1;

    // Walk up from the node: the deepest version of each agent's path wins, and the constraints are
    // collected (then put in root-to-node order).
    auto materialize = [&](int index) {
        NodeState st;
        st.index = index;
        st.paths.resize(n);
        std::vector<char> have(n, 0);
        for (int i = index; i != -1; i = nodes[static_cast<std::size_t>(i)].parent) {
            const CtNode& x = nodes[static_cast<std::size_t>(i)];
            if (x.parent != -1) st.constraints.push_back(x.constraint);
            for (const auto& [agent, path] : x.paths) {
                const auto a = static_cast<std::size_t>(agent);
                if (!have[a]) {
                    have[a] = 1;
                    st.paths[a] = path;
                }
            }
        }
        std::reverse(st.constraints.begin(), st.constraints.end());
        return st;
    };

    auto mddOf = [&](const NodeState& st, int agent) -> const Mdd& {
        // The agent's constraint set (and so its MDD) is the same from the node up to the nearest
        // ancestor that constrained it; bypass never changes a path's cost. That node holds the MDD.
        int owner = st.index;
        while (nodes[static_cast<std::size_t>(owner)].parent != -1 &&
               nodes[static_cast<std::size_t>(owner)].constraint.agent != agent) {
            owner = nodes[static_cast<std::size_t>(owner)].parent;
        }
        auto& cache = nodes[static_cast<std::size_t>(owner)].mdds;
        for (const auto& [cached, mdd] : cache) {
            if (cached == agent) return *mdd;  // shared_ptr: stays valid while the node list grows
        }
        const auto a = static_cast<std::size_t>(agent);
        cache.emplace_back(agent, std::make_shared<const Mdd>(buildMdd(grid, starts[a], goals[a],
                                                                       static_cast<int>(st.paths[a].size()) - 1,
                                                                       h[a], ConstraintTable(st.constraints, agent))));
        ++result.mddsBuilt;
        return *cache.back().second;
    };

    // How many of the two agents are forced to a longer path by their branch (0, 1 or 2).
    auto cardinality = [&](const NodeState& node, const Conflict& c) {
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

    // The two ways out of a conflict, one per agent.
    auto branchesOf = [](const Conflict& c) {
        std::array<std::pair<int, Constraint>, 2> b;
        if (c.type == ConflictType::Edge) {
            b[0] = {c.a1, edgeConstraint(c.a1, c.cell, c.to, c.t)};
            b[1] = {c.a2, edgeConstraint(c.a2, c.to, c.cell, c.t)};
        } else {
            b[0] = {c.a1, vertexConstraint(c.a1, c.cell, c.t)};
            b[1] = {c.a2, vertexConstraint(c.a2, c.cell, c.t)};
        }
        return b;
    };

    // Oracle: split the conflict for real and measure how much each child's cost goes up.
    // Returns {the cheaper child's increase, both increases added}.
    auto oracleOf = [&](const NodeState& node, const Conflict& c) {
        int increase[2] = {0, 0};
        const auto branches = branchesOf(c);
        for (std::size_t k = 0; k < 2; ++k) {
            const auto& [agent, constraint] = branches[k];
            std::vector<Constraint> constraints = node.constraints;
            constraints.push_back(constraint);
            const auto path = replan(constraints, agent);
            ++result.oracleReplans;
            const auto a = static_cast<std::size_t>(agent);
            increase[k] = path ? static_cast<int>(path->size()) - static_cast<int>(node.paths[a].size())
                               : kInfeasibleIncrease;
        }
        return std::make_pair(std::min(increase[0], increase[1]), increase[0] + increase[1]);
    };

    // Features of every conflict of a node (conflict_ranking.hpp). The per-agent counts are the
    // same for all conflicts of the node, so they are computed once.
    struct NodeCounts {
        std::vector<int> conflicts;    // per agent: conflicts of this node it is part of
        std::vector<int> constraints;  // per agent: constraints on it at this node
    };
    auto countsOf = [&](const NodeState& node, const std::vector<Conflict>& all) {
        NodeCounts counts{std::vector<int>(n, 0), std::vector<int>(n, 0)};
        for (const Conflict& c : all) {
            ++counts.conflicts[static_cast<std::size_t>(c.a1)];
            ++counts.conflicts[static_cast<std::size_t>(c.a2)];
        }
        for (const Constraint& c : node.constraints) ++counts.constraints[static_cast<std::size_t>(c.agent)];
        return counts;
    };
    auto featuresOf = [&](const NodeState& node, const Conflict& c, const NodeCounts& counts) {
        ConflictFeatureVector f{};
        auto set = [&f](ConflictFeature feature, std::int64_t value) { f[static_cast<std::size_t>(feature)] = value; };
        auto setMinMax = [&](ConflictFeature minFeature, std::int64_t x, std::int64_t y) {
            set(minFeature, std::min(x, y));
            set(static_cast<ConflictFeature>(minFeature + 1), std::max(x, y));
        };
        const auto a1 = static_cast<std::size_t>(c.a1);
        const auto a2 = static_cast<std::size_t>(c.a2);
        set(kIsVertex, c.type == ConflictType::Vertex ? 1 : 0);
        set(kIsEdge, c.type == ConflictType::Edge ? 1 : 0);
        set(kIsTarget, c.type == ConflictType::Target ? 1 : 0);
        const int forced = cardinality(node, c);
        set(kIsCardinal, forced == 2 ? 1 : 0);
        set(kIsSemiCardinal, forced == 1 ? 1 : 0);
        set(kTime, c.t);
        auto widthAt = [&](int agent) -> std::int64_t {
            const Mdd& m = mddOf(node, agent);
            if (m.levels.empty()) return 0;
            const auto level = std::min(static_cast<std::size_t>(c.t), m.levels.size() - 1);
            return static_cast<std::int64_t>(m.levels[level].size());
        };
        setMinMax(kMddWidthMin, widthAt(c.a1), widthAt(c.a2));
        setMinMax(kAgentConflictsMin, counts.conflicts[a1], counts.conflicts[a2]);
        const std::int64_t cost1 = static_cast<std::int64_t>(node.paths[a1].size()) - 1;
        const std::int64_t cost2 = static_cast<std::int64_t>(node.paths[a2].size()) - 1;
        setMinMax(kPathCostMin, cost1, cost2);
        setMinMax(kDelayMin, cost1 - h[a1][static_cast<std::size_t>(starts[a1])],
                  cost2 - h[a2][static_cast<std::size_t>(starts[a2])]);
        setMinMax(kConstraintsMin, counts.constraints[a1], counts.constraints[a2]);
        // Where each agent is at time t: in a swap, a1 has just moved to `to`, a2 to `cell`.
        const CellId at1 = c.type == ConflictType::Edge ? c.to : c.cell;
        setMinMax(kGoalDistMin, h[a1][static_cast<std::size_t>(at1)], h[a2][static_cast<std::size_t>(c.cell)]);
        return f;
    };
    auto countSplit = [&result](int forced) {
        if (forced == 2) ++result.cardinalSplits;
        else if (forced == 1) ++result.semiCardinalSplits;
        else ++result.nonCardinalSplits;
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
        NodeState node = materialize(index);
        const long long nodeCost = nodes[parentIndex].cost;

        const bool sampling = options.samples != nullptr && result.nodesExpanded <= options.sampleMaxNodes;
        const bool listAll = options.selection != ConflictSelection::First || sampling;
        std::vector<Conflict> all;
        if (listAll) all = findAllConflicts(node.paths, options.window);
        std::optional<Conflict> conflict;
        switch (options.selection) {
            case ConflictSelection::First:
                // The full list is in the same order, so its front is the first conflict.
                conflict = listAll ? (all.empty() ? std::nullopt : std::optional<Conflict>(all.front()))
                                   : findFirstConflict(node.paths, options.window);
                if (conflict) ++result.nonCardinalSplits;  // not classified
                break;
            case ConflictSelection::Cardinal: {
                int bestForced = -1;
                for (const Conflict& c : all) {  // first cardinal, else first semi-cardinal, else first
                    const int forced = cardinality(node, c);
                    if (forced > bestForced) {
                        bestForced = forced;
                        conflict = c;
                        if (forced == 2) break;
                    }
                }
                if (conflict) countSplit(bestForced);
                break;
            }
            case ConflictSelection::Learned: {
                const NodeCounts counts = countsOf(node, all);
                std::int64_t bestScore = 0;
                int bestForced = 0;
                for (const Conflict& c : all) {
                    const ConflictFeatureVector f = featuresOf(node, c, counts);
                    const std::int64_t score = options.ranker.score(f);
                    if (!conflict || score > bestScore) {
                        bestScore = score;
                        conflict = c;
                        bestForced = f[kIsCardinal] ? 2 : (f[kIsSemiCardinal] ? 1 : 0);
                    }
                }
                if (conflict) countSplit(bestForced);
                break;
            }
            case ConflictSelection::Oracle: {
                std::pair<int, int> best{0, 0};
                for (const Conflict& c : all) {
                    const auto measured = oracleOf(node, c);
                    if (!conflict || measured > best) {
                        best = measured;
                        conflict = c;
                    }
                }
                if (conflict) countSplit(cardinality(node, *conflict));
                break;
            }
        }
        if (sampling && !all.empty()) {
            const NodeCounts counts = countsOf(node, all);
            const std::size_t m = all.size();
            const std::size_t take = std::min(m, static_cast<std::size_t>(options.sampleMaxConflicts));
            for (std::size_t k = 0; k < take; ++k) {
                const std::size_t i = take == m ? k : k * m / take;  // evenly spread, deterministic
                ConflictSample sample;
                sample.node = result.nodesExpanded;
                sample.conflictIndex = static_cast<int>(i);
                sample.features = featuresOf(node, all[i], counts);
                const auto measured = oracleOf(node, all[i]);
                sample.minIncrease = measured.first;
                sample.sumIncrease = measured.second;
                options.samples->push_back(sample);
            }
        }
        if (!conflict) {
            result.solved = true;
            result.paths = std::move(node.paths);
            result.sumOfCosts = nodeCost;
            break;
        }

        const auto branches = branchesOf(*conflict);
        std::vector<CtNode> children;
        for (const auto& [agent, constraint] : branches) {
            std::vector<Constraint> constraints = node.constraints;
            constraints.push_back(constraint);
            auto path = replan(constraints, agent);
            if (!path) continue;  // this branch has no solution
            const auto a = static_cast<std::size_t>(agent);
            CtNode child;
            child.parent = index;
            child.constraint = constraint;
            // Cost and conflicts of the child's full path set: swap the new path in, then back.
            std::swap(node.paths[a], *path);
            child.cost = nodeCost + static_cast<long long>(node.paths[a].size()) - static_cast<long long>(path->size());
            child.conflicts = static_cast<int>(findAllConflicts(node.paths, options.window).size());
            std::swap(node.paths[a], *path);
            child.paths.emplace_back(agent, std::move(*path));
            children.push_back(std::move(child));
        }

        if (options.bypass) {
            // A child just as cheap with fewer conflicts: its new path also satisfies the parent's
            // constraints (a subset) and has the same length, so adopt it and look again.
            CtNode& parent = nodes[parentIndex];
            auto it = std::find_if(children.begin(), children.end(), [&](const CtNode& ch) {
                return ch.cost == parent.cost && ch.conflicts < parent.conflicts;
            });
            if (it != children.end()) {
                const int adopted = it->paths.front().first;
                Path& path = it->paths.front().second;
                auto own = std::find_if(parent.paths.begin(), parent.paths.end(),
                                        [adopted](const auto& p) { return p.first == adopted; });
                if (own != parent.paths.end()) {
                    own->second = std::move(path);
                } else {
                    parent.paths.emplace_back(adopted, std::move(path));
                }
                parent.conflicts = it->conflicts;
                // Same constraints and same cost: the agent's MDD is unchanged.
                ++result.bypasses;
                open.push(index);
                continue;
            }
        }
        for (auto& child : children) {
            nodes.push_back(std::move(child));  // may reallocate: no references held across this
            open.push(static_cast<int>(nodes.size()) - 1);
            ++result.nodesGenerated;
        }
    }
    result.seconds = elapsed();
    return result;
}

}  // namespace swarmgrid

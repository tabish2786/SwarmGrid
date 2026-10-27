#include "swarmgrid/ecbs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "swarmgrid/conflicts.hpp"
#include "swarmgrid/heuristic.hpp"

namespace swarmgrid {

// ---------------------------------------------------------------- conflict avoidance table

std::uint64_t ConflictAvoidanceTable::vertexKey(CellId cell, int t) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t)) << 32) |
           static_cast<std::uint32_t>(cell);
}

std::size_t ConflictAvoidanceTable::MoveKeyHash::operator()(const MoveKey& k) const noexcept {
    std::uint64_t x = static_cast<std::uint32_t>(k.from);
    x = x * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(k.to);
    x = x * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(k.t);
    return static_cast<std::size_t>(x ^ (x >> 29));
}

void ConflictAvoidanceTable::addPath(const Path& path, int window) {
    if (path.empty()) return;
    const int last = static_cast<int>(path.size()) - 1;
    if (window >= 0 && last > window) {
        // Parks only after the window: what happens then is not this plan's business.
        for (int t = 0; t <= window; ++t) {
            const CellId c = path[static_cast<std::size_t>(t)];
            ++moving_[vertexKey(c, t)];
            auto& times = visitTimes_[c];
            times.insert(std::upper_bound(times.begin(), times.end(), t), t);
            if (t > 0 && path[static_cast<std::size_t>(t) - 1] != c) ++moves_[MoveKey{path[static_cast<std::size_t>(t) - 1], c, t}];
        }
        return;
    }
    for (int t = 0; t < last; ++t) {
        const CellId c = path[static_cast<std::size_t>(t)];
        ++moving_[vertexKey(c, t)];
        auto& times = visitTimes_[c];
        times.insert(std::upper_bound(times.begin(), times.end(), t), t);
        const CellId next = path[static_cast<std::size_t>(t) + 1];
        if (next != c) ++moves_[MoveKey{c, next, t + 1}];
    }
    parkedFrom_[path.back()].push_back(last);
}

int ConflictAvoidanceTable::stepConflicts(CellId from, CellId to, int t) const {
    int n = 0;
    if (auto it = moving_.find(vertexKey(to, t)); it != moving_.end()) n += it->second;
    if (auto it = parkedFrom_.find(to); it != parkedFrom_.end()) {
        for (int since : it->second) n += since <= t ? 1 : 0;
    }
    if (from != to) {
        if (auto it = moves_.find(MoveKey{to, from, t}); it != moves_.end()) n += it->second;
    }
    return n;
}

int ConflictAvoidanceTable::visitsAfter(CellId cell, int t) const {
    auto it = visitTimes_.find(cell);
    if (it == visitTimes_.end()) return 0;
    return static_cast<int>(it->second.end() - std::upper_bound(it->second.begin(), it->second.end(), t));
}

// ---------------------------------------------------------------- low level: focal search

namespace {

// Largest integer cost allowed in FOCAL for a lower bound `lb`: floor(weight * lb). The small
// epsilon keeps e.g. 1.2 * 10 from rounding down to 11 in binary floating point.
long long focalBound(double weight, long long lb) {
    return static_cast<long long>(std::floor(weight * static_cast<double>(lb) + 1e-9));
}

std::uint64_t stateKey(CellId cell, int t) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t)) << 32) |
           static_cast<std::uint32_t>(cell);
}

struct FocalNode {
    CellId cell;
    int t;
    int f;
    int parent;     // index into the pool, -1 for the start
    int conflicts;  // along the path from the start (not counting what parking here would add)
    bool closed;
};

struct FocalEntry {
    int key;  // conflicts used for ordering (for a finishing state: + visits after parking)
    int f;
    int t;
    CellId cell;
    int index;
};

// true if a pops after b: fewer conflicts, then smaller f, then deeper, then smaller cell id.
struct FocalPopsLater {
    bool operator()(const FocalEntry& a, const FocalEntry& b) const {
        if (a.key != b.key) return a.key > b.key;
        if (a.f != b.f) return a.f > b.f;
        if (a.t != b.t) return a.t < b.t;
        return a.cell > b.cell;
    }
};

}  // namespace

std::optional<FocalPath> focalSpaceTimeSearch(const Grid& grid, CellId start, CellId goal,
                                              const std::vector<int>& h,
                                              const ConstraintTable& constraints,
                                              const ConflictAvoidanceTable& cat, double weight,
                                              SearchStats* stats) {
    const int n = grid.numCells();
    auto isFreeCell = [&](CellId c) { return c >= 0 && c < n && grid.isFree(c); };
    if (!isFreeCell(start) || !isFreeCell(goal)) {
        throw std::invalid_argument("focalSpaceTimeSearch: start and goal must be free cells");
    }
    if (static_cast<int>(h.size()) != n) {
        throw std::invalid_argument("focalSpaceTimeSearch: heuristic table size must equal numCells()");
    }
    if (!(weight >= 1.0)) throw std::invalid_argument("focalSpaceTimeSearch: weight must be >= 1");

    SearchStats local;
    SearchStats& s = stats != nullptr ? *stats : local;
    s = SearchStats{};

    // Same horizon and goal condition as spaceTimeAstar; see space_time_astar.cpp.
    const int limit = constraints.maxTime() + n;
    const int earliestFinish = constraints.latestVertexConstraintAt(goal) + 1;
    auto fValue = [&](CellId c, int t) { return std::max(t + h[static_cast<std::size_t>(c)], earliestFinish); };
    if (h[static_cast<std::size_t>(start)] == kUnreachable || constraints.vertexBlocked(start, 0) ||
        constraints.blockedFromAt(goal) >= 0 || fValue(start, 0) > limit) {
        return std::nullopt;
    }

    std::vector<FocalNode> pool;
    std::unordered_map<std::uint64_t, int> index;  // state -> pool index (lookups only)
    std::map<int, int> openCount;                  // f -> states generated but not expanded
    std::map<int, std::vector<int>> byF;           // f -> pool indices, to fill FOCAL as it grows
    std::priority_queue<FocalEntry, std::vector<FocalEntry>, FocalPopsLater> focal;
    long long bound = 0;

    auto keyOf = [&](const FocalNode& node) {
        const bool finishes = node.cell == goal && node.t >= earliestFinish;
        return node.conflicts + (finishes ? cat.visitsAfter(goal, node.t) : 0);
    };
    auto pushFocal = [&](int i) {
        const FocalNode& node = pool[static_cast<std::size_t>(i)];
        focal.push(FocalEntry{keyOf(node), node.f, node.t, node.cell, i});
    };

    pool.push_back(FocalNode{start, 0, fValue(start, 0), -1, 0, false});
    index[stateKey(start, 0)] = 0;
    openCount[pool[0].f] = 1;
    byF[pool[0].f].push_back(0);
    bound = focalBound(weight, pool[0].f);
    pushFocal(0);
    ++s.generated;

    std::vector<CellId> next;
    while (!focal.empty()) {
        const FocalEntry e = focal.top();
        focal.pop();
        FocalNode& top = pool[static_cast<std::size_t>(e.index)];
        if (top.closed || e.key != keyOf(top)) continue;  // stale entry
        const int fMin = openCount.begin()->first;        // lower bound before removing `top`
        top.closed = true;
        if (--openCount[top.f] == 0) openCount.erase(top.f);
        ++s.expanded;

        if (top.cell == goal && top.t >= earliestFinish) {
            FocalPath out;
            for (int i = e.index; i != -1; i = pool[static_cast<std::size_t>(i)].parent) {
                out.path.push_back(pool[static_cast<std::size_t>(i)].cell);
            }
            std::reverse(out.path.begin(), out.path.end());
            out.lowerBound = fMin;
            out.conflicts = e.key;
            return out;
        }

        const FocalNode node = top;  // copy: the pool may grow below
        grid.neighbors(node.cell, next);
        next.push_back(node.cell);  // wait
        const int t = node.t + 1;
        for (CellId c : next) {
            if (h[static_cast<std::size_t>(c)] == kUnreachable) continue;
            if (constraints.vertexBlocked(c, t)) continue;
            if (c != node.cell && constraints.edgeBlocked(node.cell, c, t)) continue;
            const int f = fValue(c, t);
            if (f > limit) continue;
            const int conflicts = node.conflicts + cat.stepConflicts(node.cell, c, t);
            const auto [it, isNew] = index.try_emplace(stateKey(c, t), static_cast<int>(pool.size()));
            if (!isNew) {
                // Same state, same g: keep it only if this way in collides less.
                FocalNode& old = pool[static_cast<std::size_t>(it->second)];
                if (old.closed || conflicts >= old.conflicts) continue;
                old.conflicts = conflicts;
                old.parent = e.index;
                if (f <= bound) pushFocal(it->second);
                continue;
            }
            pool.push_back(FocalNode{c, t, f, e.index, conflicts, false});
            ++s.generated;
            ++openCount[f];
            byF[f].push_back(it->second);
            if (f <= bound) pushFocal(it->second);
        }

        if (openCount.empty()) break;
        // The lower bound only grows (h is consistent); move the newly allowed states into FOCAL.
        const long long newBound = focalBound(weight, openCount.begin()->first);
        if (newBound > bound) {
            for (auto it = byF.upper_bound(static_cast<int>(bound));
                 it != byF.end() && it->first <= newBound; ++it) {
                for (int i : it->second) {
                    if (!pool[static_cast<std::size_t>(i)].closed) pushFocal(i);
                }
            }
            bound = newBound;
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------- high level

namespace {

struct EcbsNode {
    std::vector<Constraint> constraints;
    std::vector<Path> paths;
    std::vector<int> lowerBounds;  // per agent
    long long cost = 0;
    long long lowerBound = 0;      // sum of lowerBounds
    int conflicts = 0;
};

long long pathSum(const std::vector<Path>& paths) {
    long long s = 0;
    for (const Path& p : paths) s += static_cast<long long>(p.size()) - 1;
    return s;
}

}  // namespace

EcbsResult solveEcbs(const Grid& grid, const std::vector<CellId>& starts,
                     const std::vector<CellId>& goals, const EcbsOptions& options) {
    requireValidAgents(grid, starts, goals, "solveEcbs");
    if (options.maxNodes < 1) throw std::invalid_argument("solveEcbs: maxNodes must be >= 1");
    if (!(options.weight >= 1.0)) throw std::invalid_argument("solveEcbs: weight must be >= 1");

    using Clock = std::chrono::steady_clock;
    const auto startTime = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - startTime).count(); };

    const std::size_t n = starts.size();
    EcbsResult result;
    std::vector<std::vector<int>> h(n);
    for (std::size_t i = 0; i < n; ++i) h[i] = bfsDistances(grid, goals[i]);

    // Replan `agent` inside `node`, counting conflicts against every other agent that has a path.
    auto replan = [&](const EcbsNode& node, int agent) {
        const auto a = static_cast<std::size_t>(agent);
        ConflictAvoidanceTable cat;
        for (std::size_t j = 0; j < node.paths.size(); ++j) {
            if (j != a) cat.addPath(node.paths[j], options.window);
        }
        SearchStats stats;
        auto found = focalSpaceTimeSearch(grid, starts[a], goals[a], h[a],
                                          ConstraintTable(node.constraints, agent), cat,
                                          options.weight, &stats);
        result.lowLevelExpanded += stats.expanded;
        return found;
    };

    std::vector<EcbsNode> nodes;
    // OPEN by lower bound; FOCAL = OPEN nodes with cost <= weight * (smallest lower bound),
    // ordered by conflicts. `waiting` = OPEN nodes not yet allowed into FOCAL, by cost.
    std::set<std::pair<long long, int>> open;
    std::set<std::tuple<int, long long, int>> focal;
    std::set<std::pair<long long, int>> waiting;
    long long bound = 0;

    auto insert = [&](EcbsNode node) {
        const int id = static_cast<int>(nodes.size());
        nodes.push_back(std::move(node));
        const EcbsNode& x = nodes.back();
        open.insert({x.lowerBound, id});
        if (x.cost <= bound) {
            focal.insert({x.conflicts, x.cost, id});
        } else {
            waiting.insert({x.cost, id});
        }
        ++result.nodesGenerated;
    };
    auto raiseBound = [&] {
        if (open.empty()) return;
        const long long newBound = focalBound(options.weight, open.begin()->first);
        if (newBound <= bound) return;
        bound = newBound;
        while (!waiting.empty() && waiting.begin()->first <= bound) {
            const int id = waiting.begin()->second;
            waiting.erase(waiting.begin());
            const EcbsNode& x = nodes[static_cast<std::size_t>(id)];
            focal.insert({x.conflicts, x.cost, id});
        }
    };

    // Root: plan agents in order, each avoiding (softly) the ones already planned.
    EcbsNode root;
    root.paths.resize(n);
    root.lowerBounds.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        auto found = replan(root, static_cast<int>(i));
        if (!found) {
            result.seconds = elapsed();
            return result;
        }
        root.paths[i] = std::move(found->path);
        root.lowerBounds[i] = found->lowerBound;
    }
    root.cost = pathSum(root.paths);
    for (int lb : root.lowerBounds) root.lowerBound += lb;
    root.conflicts = static_cast<int>(findAllConflicts(root.paths, options.window).size());
    bound = focalBound(options.weight, root.lowerBound);
    insert(std::move(root));

    while (!focal.empty()) {
        if (result.nodesExpanded >= options.maxNodes ||
            (options.timeLimitSeconds > 0 && elapsed() > options.timeLimitSeconds)) {
            result.limitReached = true;
            break;
        }
        const int id = std::get<2>(*focal.begin());
        focal.erase(focal.begin());
        const long long lowerBoundNow = open.begin()->first;
        open.erase({nodes[static_cast<std::size_t>(id)].lowerBound, id});
        ++result.nodesExpanded;

        const auto conflict = findFirstConflict(nodes[static_cast<std::size_t>(id)].paths, options.window);
        if (!conflict) {
            const EcbsNode& best = nodes[static_cast<std::size_t>(id)];
            result.solved = true;
            result.paths = best.paths;
            result.sumOfCosts = best.cost;
            result.lowerBound = lowerBoundNow;
            break;
        }
        std::pair<int, Constraint> branches[2];
        if (conflict->type == ConflictType::Edge) {
            branches[0] = {conflict->a1, edgeConstraint(conflict->a1, conflict->cell, conflict->to, conflict->t)};
            branches[1] = {conflict->a2, edgeConstraint(conflict->a2, conflict->to, conflict->cell, conflict->t)};
        } else {
            branches[0] = {conflict->a1, vertexConstraint(conflict->a1, conflict->cell, conflict->t)};
            branches[1] = {conflict->a2, vertexConstraint(conflict->a2, conflict->cell, conflict->t)};
        }
        for (const auto& [agent, constraint] : branches) {
            EcbsNode child = nodes[static_cast<std::size_t>(id)];
            child.constraints.push_back(constraint);
            auto found = replan(child, agent);
            if (!found) continue;
            const auto a = static_cast<std::size_t>(agent);
            child.paths[a] = std::move(found->path);
            // More constraints never make the true optimum smaller, so the old bound still holds.
            child.lowerBounds[a] = std::max(child.lowerBounds[a], found->lowerBound);
            child.cost = pathSum(child.paths);
            child.lowerBound = 0;
            for (int lb : child.lowerBounds) child.lowerBound += lb;
            child.conflicts = static_cast<int>(findAllConflicts(child.paths, options.window).size());
            insert(std::move(child));
        }
        raiseBound();
    }
    result.seconds = elapsed();
    return result;
}

}  // namespace swarmgrid

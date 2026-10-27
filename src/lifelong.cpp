#include "swarmgrid/lifelong.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "swarmgrid/cbs.hpp"
#include "swarmgrid/ecbs.hpp"
#include "swarmgrid/heuristic.hpp"
#include "swarmgrid/prioritized.hpp"
#include "swarmgrid/task_assignment.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace swarmgrid {

const char* solverName(LifelongSolver s) {
    switch (s) {
        case LifelongSolver::Prioritized: return "prioritized";
        case LifelongSolver::Cbs: return "cbs";
        case LifelongSolver::Ecbs: return "ecbs";
    }
    return "?";
}

const char* assignName(AssignPolicy a) { return a == AssignPolicy::Greedy ? "greedy" : "hungarian"; }

namespace {

enum class Phase { Free, ToPickup, ToStation };

struct Robot {
    CellId pos = kInvalidCell;
    Phase phase = Phase::Free;
    CellId pickup = kInvalidCell;    // ToPickup: the order's cell
    CellId station = kInvalidCell;   // ToStation: kInvalidCell until one is free
    bool broken = false;
    int repairAt = 0;
    bool escaping = false;
    CellId escapeGoal = kInvalidCell;
    int escapeUntil = 0;
    CellId goal = kInvalidCell;      // planning goal of the current plan
    int bestDist = std::numeric_limits<int>::max();
    int lastProgress = 0;
};

class Simulator {
public:
    Simulator(const Grid& grid, const LifelongOptions& o) : base_(grid), o_(o), rng_(o.seed) {
        for (CellId c = 0; c < grid.numCells(); ++c) {
            if (!grid.isFree(c)) continue;
            const Point p = grid.toPoint(c);
            if (p.x == 0 || p.y == 0 || p.x == grid.width() - 1 || p.y == grid.height() - 1) {
                stations_.push_back(c);
                continue;
            }
            std::vector<CellId> nb;
            grid.neighbors(c, nb);
            if (nb.size() < 4) pickups_.push_back(c);  // next to a shelf (or wall)
        }
        if (stations_.empty() || pickups_.empty()) {
            throw std::invalid_argument("runLifelong: the map needs free cells on its outer ring "
                                        "(stations) and free cells next to shelves (pickups)");
        }
        const MapfInstance inst = generateRandomInstance(grid, o.robots, o.seed);
        robots_.resize(static_cast<std::size_t>(o.robots));
        for (std::size_t i = 0; i < robots_.size(); ++i) robots_[i].pos = grid.toId(inst.starts[i]);
        backlog_ = o.backlog > 0 ? o.backlog : o.robots;
    }

    LifelongStats run(bool record) {
        if (record) stats_.positions.push_back(positions());
        int planStep = -1;
        for (int step = 0; step < o_.steps; ++step) {
            bool changed = breakdownsAndRepairs(step);
            for (Robot& r : robots_) {
                if (r.escaping && step >= r.escapeUntil) {  // escape took too long: resume the task
                    r.escaping = false;
                    changed = true;
                }
            }
            if (changed || planStep < 0 || step - planStep >= o_.replanEvery) {
                replan(step);
                planStep = step;
            }
            execute(step - planStep);
            arrivals();
            if (record) stats_.positions.push_back(positions());
        }
        stats_.steps = o_.steps;
        stats_.throughput = static_cast<double>(stats_.delivered) / o_.steps;
        if (stats_.replans > 0) stats_.meanPlanSeconds = planSeconds_ / stats_.replans;
        return stats_;
    }

private:
    std::vector<CellId> positions() const {
        std::vector<CellId> out;
        for (const Robot& r : robots_) out.push_back(r.pos);
        return out;
    }

    bool chance(double p) {
        if (p <= 0) return false;
        return rng_.uniformInt(0, 999999) < static_cast<int>(p * 1e6);
    }

    bool breakdownsAndRepairs(int step) {
        bool changed = false;
        for (Robot& r : robots_) {
            if (r.broken && step >= r.repairAt) {
                r.broken = false;
                r.bestDist = std::numeric_limits<int>::max();
                changed = true;
            } else if (!r.broken && chance(o_.breakdownRate)) {
                r.broken = true;
                r.repairAt = step + o_.repairSteps;
                r.escaping = false;
                if (r.phase == Phase::ToPickup) {  // not picked up yet: someone else can take it
                    waiting_.push_back(r.pickup);
                    r.phase = Phase::Free;
                    r.pickup = kInvalidCell;
                }
                ++stats_.breakdowns;
                changed = true;
            }
        }
        return changed;
    }

    // Cells another task may not use as its goal.
    std::unordered_set<CellId> reservedCells(bool includePositions) const {
        std::unordered_set<CellId> s(waiting_.begin(), waiting_.end());
        for (const Robot& r : robots_) {
            if (r.phase == Phase::ToPickup) s.insert(r.pickup);
            if (r.phase == Phase::ToStation && r.station != kInvalidCell) s.insert(r.station);
            if (r.escaping) s.insert(r.escapeGoal);
            if (includePositions || r.broken) s.insert(r.pos);
        }
        return s;
    }

    void refillOrders(const Grid& plan) {
        auto taken = reservedCells(true);
        int tries = 0;
        while (static_cast<int>(waiting_.size()) < backlog_ && tries < 1000) {
            ++tries;
            const CellId c = pickups_[static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(pickups_.size()) - 1))];
            if (taken.count(c) || !plan.isFree(c)) continue;
            waiting_.push_back(c);
            taken.insert(c);
        }
    }

    CellId pickStation(const Grid& plan) {
        const auto taken = reservedCells(true);
        for (int tries = 0; tries < 50; ++tries) {
            const CellId c = stations_[static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(stations_.size()) - 1))];
            if (!taken.count(c) && plan.isFree(c)) return c;
        }
        for (CellId c : stations_) {
            if (!taken.count(c) && plan.isFree(c)) return c;
        }
        return kInvalidCell;  // every station busy: wait and try at the next replan
    }

    CellId taskGoal(const Robot& r) const {
        if (r.escaping) return r.escapeGoal;
        if (r.phase == Phase::ToPickup) return r.pickup;
        if (r.phase == Phase::ToStation && r.station != kInvalidCell) return r.station;
        return kInvalidCell;  // free, or waiting for a station: holds a cell
    }

    void replan(int step) {
        ++stats_.replans;
        Grid plan = base_;
        for (const Robot& r : robots_) {
            if (r.broken) plan.setObstacle(plan.toPoint(r.pos).x, plan.toPoint(r.pos).y, true);
        }

        // 1. Goals that a breakdown made unusable (blocked or cut off).
        for (Robot& r : robots_) {
            if (r.broken) continue;
            const CellId g = taskGoal(r);
            if (g == kInvalidCell) continue;
            if (plan.isFree(g) && bfsDistances(plan, g)[static_cast<std::size_t>(r.pos)] != kUnreachable) continue;
            if (r.escaping) {
                r.escaping = false;
            } else if (r.phase == Phase::ToPickup) {
                waiting_.push_back(r.pickup);
                r.phase = Phase::Free;
            } else {
                r.station = kInvalidCell;
            }
        }

        // 2. Orders for free robots.
        refillOrders(plan);
        std::vector<int> freeRobots;
        std::vector<CellId> freeCells;
        for (std::size_t i = 0; i < robots_.size(); ++i) {
            const Robot& r = robots_[i];
            if (!r.broken && !r.escaping && r.phase == Phase::Free) {
                freeRobots.push_back(static_cast<int>(i));
                freeCells.push_back(r.pos);
            }
        }
        std::vector<CellId> open;  // waiting orders a robot can stand on now
        std::vector<std::size_t> openIndex;
        for (std::size_t j = 0; j < waiting_.size(); ++j) {
            if (plan.isFree(waiting_[j])) {
                open.push_back(waiting_[j]);
                openIndex.push_back(j);
            }
        }
        if (!freeRobots.empty() && !open.empty()) {
            const auto pairs = o_.assignment == AssignPolicy::Greedy ? assignNearest(plan, freeCells, open)
                                                                     : assignOptimal(plan, freeCells, open);
            std::vector<bool> takenOrder(waiting_.size(), false);
            for (const Assignment& a : pairs) {
                Robot& r = robots_[static_cast<std::size_t>(freeRobots[static_cast<std::size_t>(a.robot)])];
                r.phase = Phase::ToPickup;
                r.pickup = open[static_cast<std::size_t>(a.order)];
                r.bestDist = std::numeric_limits<int>::max();
                takenOrder[openIndex[static_cast<std::size_t>(a.order)]] = true;
                stats_.assignedDistance += a.distance;
            }
            std::vector<CellId> rest;
            for (std::size_t j = 0; j < waiting_.size(); ++j) {
                if (!takenOrder[j]) rest.push_back(waiting_[j]);
            }
            waiting_ = std::move(rest);
        }
        for (Robot& r : robots_) {
            if (!r.broken && r.phase == Phase::ToStation && r.station == kInvalidCell) {
                r.station = pickStation(plan);
                r.bestDist = std::numeric_limits<int>::max();
            }
        }
        refillOrders(plan);

        // 3. Deadlock / livelock detection: no progress towards the goal for stuckSteps.
        int newlyStuck = 0;
        for (Robot& r : robots_) {
            if (r.broken) continue;
            const CellId g = taskGoal(r);
            if (g == kInvalidCell) {
                r.bestDist = 0;
                r.lastProgress = step;
                continue;
            }
            const int d = bfsDistances(plan, g)[static_cast<std::size_t>(r.pos)];
            if (d < r.bestDist) {
                r.bestDist = d;
                r.lastProgress = step;
            }
            if (d > 0 && step - r.lastProgress >= o_.stuckSteps && !r.escaping) {
                ++newlyStuck;
                const CellId escape = pickEscape(plan, r.pos);
                r.lastProgress = step;
                r.bestDist = std::numeric_limits<int>::max();
                if (escape != kInvalidCell) {
                    r.escaping = true;
                    r.escapeGoal = escape;
                    r.escapeUntil = step + 3 * std::max(o_.replanEvery, o_.window) + 10;
                    ++stats_.escapes;
                }
            }
        }
        if (newlyStuck > 0) ++stats_.deadlocks;

        // 4. Final goals: task goal, or hold the current cell (or the nearest unclaimed one).
        std::unordered_set<CellId> goalSet;
        for (const Robot& r : robots_) {
            if (!r.broken && taskGoal(r) != kInvalidCell) goalSet.insert(taskGoal(r));
        }
        std::vector<int> active;
        std::vector<CellId> starts, goals;
        for (std::size_t i = 0; i < robots_.size(); ++i) {
            Robot& r = robots_[i];
            if (r.broken) continue;
            CellId g = taskGoal(r);
            if (g == kInvalidCell) {
                g = goalSet.count(r.pos) ? nearestUnclaimed(plan, r.pos, goalSet) : r.pos;
                goalSet.insert(g);
            }
            r.goal = g;
            active.push_back(static_cast<int>(i));
            starts.push_back(r.pos);
            goals.push_back(g);
        }

        // 5. Plan.
        plan_.assign(robots_.size(), Path{});
        for (std::size_t i = 0; i < robots_.size(); ++i) plan_[i] = Path{robots_[i].pos};
        if (active.empty()) return;
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<Path> paths;
        bool solved = false;
        if (o_.solver == LifelongSolver::Ecbs) {
            EcbsOptions e;
            e.weight = o_.ecbsWeight;
            e.maxNodes = o_.plannerMaxNodes;
            e.timeLimitSeconds = o_.plannerTimeLimit;
            e.window = o_.window;
            EcbsResult r = solveEcbs(plan, starts, goals, e);
            solved = r.solved;
            paths = std::move(r.paths);
        } else if (o_.solver == LifelongSolver::Cbs) {
            CbsOptions c;
            c.maxNodes = o_.plannerMaxNodes;
            c.timeLimitSeconds = o_.plannerTimeLimit;
            c.window = o_.window;
            CbsResult r = solveCbs(plan, starts, goals, c);
            solved = r.solved;
            paths = std::move(r.paths);
        }
        if (!solved) {
            if (o_.solver != LifelongSolver::Prioritized) ++stats_.plannerFailures;
            MultiAgentResult r = planMultiAgent(plan, starts, goals, MultiAgentMode::Prioritized, 10, o_.window);
            solved = r.solved;
            paths = std::move(r.paths);
            if (!solved && o_.solver == LifelongSolver::Prioritized) ++stats_.plannerFailures;
        }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        planSeconds_ += secs;
        stats_.maxPlanSeconds = std::max(stats_.maxPlanSeconds, secs);
        if (!solved) {
            ++stats_.allWait;  // plan_ already says "everyone stays where it is"
            return;
        }
        for (std::size_t j = 0; j < active.size(); ++j) {
            plan_[static_cast<std::size_t>(active[j])] = std::move(paths[j]);
        }
    }

    CellId pickEscape(const Grid& plan, CellId from) {
        const auto taken = reservedCells(true);
        const auto dist = bfsDistances(plan, from);
        std::vector<CellId> options;
        for (CellId c = 0; c < plan.numCells(); ++c) {
            const int d = dist[static_cast<std::size_t>(c)];
            if (d >= 2 && d <= 6 && !taken.count(c)) options.push_back(c);
        }
        if (options.empty()) return kInvalidCell;
        return options[static_cast<std::size_t>(rng_.uniformInt(0, static_cast<int>(options.size()) - 1))];
    }

    // Closest free cell (BFS order, ties by the grid's fixed neighbour order) nobody claims.
    static CellId nearestUnclaimed(const Grid& plan, CellId from, const std::unordered_set<CellId>& claimed) {
        std::vector<int> seen(static_cast<std::size_t>(plan.numCells()), 0);
        std::vector<CellId> queue{from}, nb;
        seen[static_cast<std::size_t>(from)] = 1;
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const CellId c = queue[head];
            if (!claimed.count(c)) return c;
            plan.neighbors(c, nb);
            for (CellId m : nb) {
                if (!seen[static_cast<std::size_t>(m)]) {
                    seen[static_cast<std::size_t>(m)] = 1;
                    queue.push_back(m);
                }
            }
        }
        throw std::logic_error("runLifelong: no free cell left to park a robot");
    }

    void execute(int offset) {
        std::vector<CellId> next(robots_.size());
        for (std::size_t i = 0; i < robots_.size(); ++i) {
            const Path& p = plan_[i];
            next[i] = robots_[i].broken ? robots_[i].pos
                                        : p[std::min(static_cast<std::size_t>(offset) + 1, p.size() - 1)];
        }
        // Independent check of what actually happened: same cell, or two robots swapping.
        for (std::size_t i = 0; i < robots_.size(); ++i) {
            for (std::size_t j = i + 1; j < robots_.size(); ++j) {
                if (next[i] == next[j]) ++stats_.collisions;
                if (next[i] == robots_[j].pos && next[j] == robots_[i].pos && next[i] != next[j]) ++stats_.collisions;
            }
            if (!base_.isFree(next[i])) ++stats_.collisions;
        }
        for (std::size_t i = 0; i < robots_.size(); ++i) robots_[i].pos = next[i];
    }

    void arrivals() {
        for (Robot& r : robots_) {
            if (r.broken) continue;
            if (r.escaping) {
                if (r.pos == r.escapeGoal) {
                    r.escaping = false;
                    r.bestDist = std::numeric_limits<int>::max();
                }
                continue;
            }
            if (r.phase == Phase::ToPickup && r.pos == r.pickup) {
                r.phase = Phase::ToStation;
                r.station = kInvalidCell;  // chosen at the next replan
                r.bestDist = std::numeric_limits<int>::max();
            } else if (r.phase == Phase::ToStation && r.station != kInvalidCell && r.pos == r.station) {
                r.phase = Phase::Free;
                r.station = kInvalidCell;
                ++stats_.delivered;
            }
        }
    }

    const Grid& base_;
    LifelongOptions o_;
    Rng rng_;
    std::vector<CellId> stations_, pickups_, waiting_;
    std::vector<Robot> robots_;
    std::vector<Path> plan_;
    int backlog_ = 0;
    double planSeconds_ = 0.0;
    LifelongStats stats_;
};

}  // namespace

LifelongStats runLifelong(const Grid& grid, const LifelongOptions& options, bool recordPositions) {
    if (options.robots < 1 || options.steps < 1) {
        throw std::invalid_argument("runLifelong: robots and steps must be >= 1");
    }
    if (options.replanEvery < 1 || (options.window >= 0 && options.window < options.replanEvery) ||
        options.window < -1) {
        throw std::invalid_argument("runLifelong: need 1 <= replanEvery <= window (or window = -1)");
    }
    if (options.plannerMaxNodes < 1 || !(options.ecbsWeight >= 1.0) || options.repairSteps < 1 ||
        options.stuckSteps < 1 || options.breakdownRate < 0 || options.breakdownRate > 1) {
        throw std::invalid_argument("runLifelong: bad planner, breakdown or stuck settings");
    }
    Simulator sim(grid, options);
    return sim.run(recordPositions);
}

}  // namespace swarmgrid

#include "swarmgrid/conflicts.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <unordered_set>

namespace swarmgrid {

namespace {

CellId at(const Path& p, std::size_t t) { return t < p.size() ? p[t] : p.back(); }

std::size_t horizonOf(const std::vector<Path>& paths) {
    std::size_t h = 0;
    for (const Path& p : paths) h = std::max(h, p.size());
    return h;
}

// Conflicts at exactly time t, in the documented order. Stops after the first if firstOnly.
void conflictsAt(const std::vector<Path>& paths, std::size_t t, bool firstOnly,
                 std::vector<Conflict>& out) {
    const std::size_t n = paths.size();
    const int time = static_cast<int>(t);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (at(paths[i], t) != at(paths[j], t)) continue;
            Conflict c;
            c.cell = at(paths[i], t);
            c.t = time;
            const bool iParked = t + 1 >= paths[i].size();  // reached the end of its path
            const bool jParked = t + 1 >= paths[j].size();
            if (iParked || jParked) {
                c.type = ConflictType::Target;
                c.a1 = static_cast<int>(iParked ? i : j);  // the parked one
                c.a2 = static_cast<int>(iParked ? j : i);
            } else {
                c.type = ConflictType::Vertex;
                c.a1 = static_cast<int>(i);
                c.a2 = static_cast<int>(j);
            }
            out.push_back(c);
            if (firstOnly) return;
        }
    }
    if (t == 0) return;
    for (std::size_t i = 0; i < n; ++i) {
        const CellId from = at(paths[i], t - 1);
        const CellId to = at(paths[i], t);
        if (from == to) continue;
        for (std::size_t j = i + 1; j < n; ++j) {
            if (at(paths[j], t - 1) == to && at(paths[j], t) == from) {
                Conflict c;
                c.type = ConflictType::Edge;
                c.a1 = static_cast<int>(i);
                c.a2 = static_cast<int>(j);
                c.cell = from;
                c.to = to;
                c.t = time;
                out.push_back(c);
                if (firstOnly) return;
            }
        }
    }
}

}  // namespace

std::optional<Conflict> findFirstConflict(const std::vector<Path>& paths) {
    std::vector<Conflict> found;
    const std::size_t horizon = horizonOf(paths);
    for (std::size_t t = 0; t < horizon; ++t) {
        conflictsAt(paths, t, true, found);
        if (!found.empty()) return found.front();
    }
    return std::nullopt;  // after the horizon everyone is parked on distinct cells or conflicting already
}

std::vector<Conflict> findAllConflicts(const std::vector<Path>& paths) {
    std::vector<Conflict> found;
    const std::size_t horizon = horizonOf(paths);
    for (std::size_t t = 0; t < horizon; ++t) conflictsAt(paths, t, false, found);
    return found;
}

ValidationReport validatePlan(const Grid& grid, const std::vector<CellId>& starts,
                              const std::vector<CellId>& goals, const std::vector<Path>& paths) {
    ValidationReport r;
    if (starts.size() != goals.size() || paths.size() != starts.size()) {
        r.errors.push_back("expected one path per agent");
        return r;
    }
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const Path& p = paths[i];
        const std::string who = "agent " + std::to_string(i) + ": ";
        if (p.empty()) {
            r.errors.push_back(who + "empty path");
            continue;
        }
        if (p.front() != starts[i]) r.errors.push_back(who + "does not start at its start");
        if (p.back() != goals[i]) r.errors.push_back(who + "does not end at its goal");
        for (std::size_t t = 0; t < p.size(); ++t) {
            if (p[t] < 0 || p[t] >= grid.numCells() || !grid.isFree(p[t])) {
                r.errors.push_back(who + "on an obstacle or outside the grid at t=" + std::to_string(t));
                break;
            }
            if (t > 0) {
                const Point a = grid.toPoint(p[t - 1]);
                const Point b = grid.toPoint(p[t]);
                if (std::abs(a.x - b.x) + std::abs(a.y - b.y) > 1) {
                    r.errors.push_back(who + "jumps more than one cell at t=" + std::to_string(t));
                    break;
                }
            }
        }
        const auto cost = static_cast<long long>(p.size()) - 1;
        r.sumOfCosts += cost;
        r.makespan = std::max(r.makespan, static_cast<int>(cost));
    }
    if (r.errors.empty()) r.conflicts = findAllConflicts(paths);
    r.ok = r.errors.empty() && r.conflicts.empty();
    return r;
}

void requireValidAgents(const Grid& grid, const std::vector<CellId>& starts,
                        const std::vector<CellId>& goals, const std::string& who) {
    if (starts.empty() || starts.size() != goals.size()) {
        throw std::invalid_argument(who + ": need the same, non-zero number of starts and goals");
    }
    std::unordered_set<CellId> seenStarts;
    std::unordered_set<CellId> seenGoals;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        for (CellId c : {starts[i], goals[i]}) {
            if (c < 0 || c >= grid.numCells() || !grid.isFree(c)) {
                throw std::invalid_argument(who + ": agent " + std::to_string(i) +
                                            " has a start or goal that is not a free cell");
            }
        }
        if (!seenStarts.insert(starts[i]).second || !seenGoals.insert(goals[i]).second) {
            throw std::invalid_argument(who + ": agent " + std::to_string(i) +
                                        " shares its start or goal with another agent");
        }
    }
}

}  // namespace swarmgrid

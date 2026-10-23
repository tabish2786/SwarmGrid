#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

enum class ConstraintType { Vertex, Edge, BlockedFrom };

// A CBS constraint on one agent.
//   Vertex:      the agent may not be at `from` at time t.
//   Edge:        the agent may not move from `from` (at time t-1) to `to` (at time t).
//   BlockedFrom: the agent may not be at `from` at ANY time >= t. Used when another robot parks
//                on its goal `from` at time t and stays there (prioritized planning, and the
//                target-conflict branch of CBS).
// Build them with the factory functions so the fields are always consistent.
struct Constraint {
    ConstraintType type = ConstraintType::Vertex;
    int agent = 0;
    CellId from = kInvalidCell;
    CellId to = kInvalidCell;  // Edge only; kInvalidCell for Vertex
    int t = 0;
};

// Throw std::invalid_argument on a negative cell, t < 0 (vertex), t < 1 or from == to (edge).
// A wait is not an edge, so it can only be forbidden with a vertex constraint.
Constraint vertexConstraint(int agent, CellId cell, int t);
Constraint edgeConstraint(int agent, CellId from, CellId to, int t);
// Throws std::invalid_argument on a negative cell or t < 0.
Constraint blockedFromConstraint(int agent, CellId cell, int t);

// O(1) expected-time lookups of one agent's constraints, used by the low-level search.
class ConstraintTable {
public:
    ConstraintTable() = default;

    // Keeps only the constraints whose agent == `agent`.
    ConstraintTable(const std::vector<Constraint>& constraints, int agent);

    // Adds c regardless of c.agent.
    void add(const Constraint& c);

    // True for a vertex constraint at exactly t, or a BlockedFrom constraint starting at or before t.
    bool vertexBlocked(CellId cell, int t) const;
    // Is the move from `from` at time t-1 to `to` at time t forbidden?
    bool edgeBlocked(CellId from, CellId to, int t) const;

    // Largest t of a Vertex constraint on `cell`, or -1 if there is none. An agent can only
    // finish at its goal at a time T > latestVertexConstraintAt(goal); otherwise it would be
    // standing on the goal when that constraint applies. BlockedFrom is reported separately.
    int latestVertexConstraintAt(CellId cell) const;

    // Earliest t of a BlockedFrom constraint on `cell`, or -1 if there is none. An agent whose
    // goal has one can never finish there (it would have to stay forever).
    int blockedFromAt(CellId cell) const;

    // Largest t over all constraints (including BlockedFrom start times), or -1 if empty.
    int maxTime() const { return maxTime_; }
    bool empty() const { return maxTime_ < 0; }

private:
    // Vertex keys pack (t, cell) into one 64-bit integer: no custom hash needed.
    static std::uint64_t vertexKey(CellId cell, int t);

    // Edge keys have three ints, which don't fit losslessly in 64 bits, so they get a struct
    // with operator== and a hash functor.
    struct EdgeKey {
        CellId from;
        CellId to;
        int t;
        bool operator==(const EdgeKey& o) const { return from == o.from && to == o.to && t == o.t; }
    };
    struct EdgeKeyHash {
        std::size_t operator()(const EdgeKey& k) const noexcept;
    };

    // Unordered containers are only used for lookups, never iterated, so they cannot make the
    // search order depend on the standard library implementation.
    std::unordered_set<std::uint64_t> vertex_;
    std::unordered_set<EdgeKey, EdgeKeyHash> edge_;
    std::unordered_map<CellId, int> latestVertex_;
    std::unordered_map<CellId, int> blockedFrom_;  // cell -> earliest time it is blocked forever
    int maxTime_ = -1;
};

}  // namespace swarmgrid

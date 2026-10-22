#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "swarmgrid/grid.hpp"

namespace swarmgrid {

enum class ConstraintType { Vertex, Edge };

// A CBS constraint on one agent.
//   Vertex: the agent may not be at `from` at time t.
//   Edge:   the agent may not move from `from` (at time t-1) to `to` (at time t).
// Build them with vertexConstraint() / edgeConstraint() so the fields are always consistent.
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

// O(1) expected-time lookups of one agent's constraints, used by the low-level search.
class ConstraintTable {
public:
    ConstraintTable() = default;

    // Keeps only the constraints whose agent == `agent`.
    ConstraintTable(const std::vector<Constraint>& constraints, int agent);

    // Adds c regardless of c.agent.
    void add(const Constraint& c);

    bool vertexBlocked(CellId cell, int t) const;
    // Is the move from `from` at time t-1 to `to` at time t forbidden?
    bool edgeBlocked(CellId from, CellId to, int t) const;

    // Largest t of a vertex constraint on `cell`, or -1 if there is none. An agent can only
    // finish at its goal at a time T > latestVertexConstraintAt(goal); otherwise it would be
    // standing on the goal when that constraint applies.
    int latestVertexConstraintAt(CellId cell) const;

    // Largest t over all constraints, or -1 if the table is empty.
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
    int maxTime_ = -1;
};

}  // namespace swarmgrid

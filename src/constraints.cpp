#include "swarmgrid/constraints.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace swarmgrid {

Constraint vertexConstraint(int agent, CellId cell, int t) {
    if (cell < 0 || t < 0) {
        throw std::invalid_argument("vertexConstraint: need cell >= 0 and t >= 0");
    }
    Constraint c;
    c.type = ConstraintType::Vertex;
    c.agent = agent;
    c.from = cell;
    c.to = kInvalidCell;
    c.t = t;
    return c;
}

Constraint edgeConstraint(int agent, CellId from, CellId to, int t) {
    if (from < 0 || to < 0 || from == to || t < 1) {
        throw std::invalid_argument(
            "edgeConstraint: need distinct cells >= 0 and t >= 1 (a wait is not an edge)");
    }
    Constraint c;
    c.type = ConstraintType::Edge;
    c.agent = agent;
    c.from = from;
    c.to = to;
    c.t = t;
    return c;
}

ConstraintTable::ConstraintTable(const std::vector<Constraint>& constraints, int agent) {
    for (const Constraint& c : constraints) {
        if (c.agent == agent) {
            add(c);
        }
    }
}

void ConstraintTable::add(const Constraint& c) {
    if (c.type == ConstraintType::Vertex) {
        // Re-validate: a Constraint can also be built by hand.
        const Constraint v = vertexConstraint(c.agent, c.from, c.t);
        vertex_.insert(vertexKey(v.from, v.t));
        auto it = latestVertex_.find(v.from);
        if (it == latestVertex_.end()) {
            latestVertex_.emplace(v.from, v.t);
        } else {
            it->second = std::max(it->second, v.t);
        }
    } else {
        const Constraint e = edgeConstraint(c.agent, c.from, c.to, c.t);
        edge_.insert(EdgeKey{e.from, e.to, e.t});
    }
    maxTime_ = std::max(maxTime_, c.t);
}

bool ConstraintTable::vertexBlocked(CellId cell, int t) const {
    if (cell < 0 || t < 0) {
        return false;  // no constraint can exist there (add() rejects such constraints)
    }
    return vertex_.count(vertexKey(cell, t)) > 0;
}

bool ConstraintTable::edgeBlocked(CellId from, CellId to, int t) const {
    return edge_.count(EdgeKey{from, to, t}) > 0;
}

int ConstraintTable::latestVertexConstraintAt(CellId cell) const {
    const auto it = latestVertex_.find(cell);
    return it == latestVertex_.end() ? -1 : it->second;
}

std::uint64_t ConstraintTable::vertexKey(CellId cell, int t) {
    // Both are >= 0 here, so each fits in 32 bits: t in the high half, cell in the low half.
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t)) << 32) |
           static_cast<std::uint32_t>(cell);
}

std::size_t ConstraintTable::EdgeKeyHash::operator()(const EdgeKey& k) const noexcept {
    // Mix the three fields with multiply-xor (constant = 2^64 / golden ratio). A plain
    // from ^ to ^ t would collide for every permutation of the same three numbers.
    std::uint64_t h = static_cast<std::uint32_t>(k.from);
    h = h * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(k.to);
    h = h * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(k.t);
    return static_cast<std::size_t>(h ^ (h >> 32));
}

}  // namespace swarmgrid

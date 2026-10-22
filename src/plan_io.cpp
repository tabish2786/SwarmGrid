#include "swarmgrid/plan_io.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace swarmgrid {

namespace {

const char* const kHexDigits = "0123456789abcdef";

void requireInside(const Grid& grid, Point p, const char* what) {
    if (!grid.inBounds(p.x, p.y)) {
        throw std::invalid_argument(std::string("writePlanJson: ") + what + " is outside the grid");
    }
}

void requireInside(const Grid& grid, CellId c, const char* what) {
    if (c < 0 || c >= grid.numCells()) {
        throw std::invalid_argument(std::string("writePlanJson: ") + what + " is outside the grid");
    }
}

void writeXY(std::ostream& out, Point p) { out << '[' << p.x << ',' << p.y << ']'; }

}  // namespace

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {  // other control characters must be \u-escaped in JSON
                    out += "\\u00";
                    out += kHexDigits[c >> 4];
                    out += kHexDigits[c & 0xF];
                } else {
                    out += ch;  // UTF-8 bytes pass through unchanged
                }
        }
    }
    return out;
}

void writePlanJson(std::ostream& out, const PlanExport& plan) {
    const Grid& g = plan.grid;
    const std::size_t n = plan.paths.size();
    if (plan.starts.size() != n || plan.goals.size() != n ||
        (!plan.expanded.empty() && plan.expanded.size() != n)) {
        throw std::invalid_argument(
            "writePlanJson: starts, goals, paths (and expanded, if given) need the same size");
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (plan.paths[i].empty()) {
            throw std::invalid_argument("writePlanJson: agent " + std::to_string(i) +
                                        " has an empty path");
        }
        requireInside(g, plan.starts[i], "a start");
        requireInside(g, plan.goals[i], "a goal");
        for (CellId c : plan.paths[i]) requireInside(g, c, "a path cell");
    }
    for (const Constraint& c : plan.constraints) {
        requireInside(g, c.from, "a constraint cell");
        if (c.type == ConstraintType::Edge) requireInside(g, c.to, "a constraint cell");
    }

    out << "{\"format\":\"swarmgrid-plan-v1\",\"id\":\"" << jsonEscape(plan.id) << "\",\"title\":\""
        << jsonEscape(plan.title) << "\",\"solver\":\"" << jsonEscape(plan.solver)
        << "\",\"note\":\"" << jsonEscape(plan.note) << "\",\n";

    out << " \"map\":{\"width\":" << g.width() << ",\"height\":" << g.height() << ",\"rows\":[";
    for (int y = 0; y < g.height(); ++y) {
        out << (y > 0 ? "," : "") << "\n  \"";
        for (int x = 0; x < g.width(); ++x) out << (g.isFree(x, y) ? '.' : '@');
        out << '"';
    }
    out << "]},\n \"agents\":[";

    long long sumOfCosts = 0;
    long long makespan = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Path& path = plan.paths[i];
        const auto cost = static_cast<long long>(path.size()) - 1;
        sumOfCosts += cost;
        makespan = std::max(makespan, cost);

        out << (i > 0 ? "," : "") << "\n  {\"id\":" << i << ",\"start\":";
        writeXY(out, plan.starts[i]);
        out << ",\"goal\":";
        writeXY(out, plan.goals[i]);
        out << ",\"cost\":" << cost;
        if (!plan.expanded.empty()) out << ",\"expanded\":" << plan.expanded[i];
        out << ",\"path\":[";
        for (std::size_t t = 0; t < path.size(); ++t) {
            const Point p = g.toPoint(path[t]);
            out << (t > 0 ? "," : "") << '[' << p.x << ',' << p.y << ',' << t << ']';
        }
        out << "]}";
    }
    out << "],\n \"constraints\":[";
    for (std::size_t i = 0; i < plan.constraints.size(); ++i) {
        const Constraint& c = plan.constraints[i];
        out << (i > 0 ? "," : "") << "\n  {\"agent\":" << c.agent << ",\"type\":\""
            << (c.type == ConstraintType::Vertex ? "vertex" : "edge") << "\",\"cell\":";
        writeXY(out, g.toPoint(c.from));
        if (c.type == ConstraintType::Edge) {
            out << ",\"to\":";
            writeXY(out, g.toPoint(c.to));
        }
        out << ",\"t\":" << c.t << '}';
    }
    out << "],\n \"stats\":{\"agents\":" << n << ",\"sum_of_costs\":" << sumOfCosts
        << ",\"makespan\":" << makespan << "}}";
}

}  // namespace swarmgrid

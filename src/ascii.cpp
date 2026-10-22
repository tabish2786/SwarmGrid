#include "swarmgrid/ascii.hpp"

#include <stdexcept>

namespace swarmgrid {

std::string renderGrid(const Grid& grid) {
    std::string out;
    // Each row is width characters plus '\n'. Reserving once avoids repeated reallocation.
    out.reserve(static_cast<std::size_t>((grid.width() + 1) * grid.height()));
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            out += grid.isFree(x, y) ? '.' : '@';
        }
        out += '\n';
    }
    return out;
}

char agentSymbol(int agentIndex) {
    if (agentIndex < 0) {
        throw std::invalid_argument("agentSymbol: negative agent index");
    }
    if (agentIndex < 10) return static_cast<char>('0' + agentIndex);
    if (agentIndex < 36) return static_cast<char>('a' + (agentIndex - 10));
    if (agentIndex < 62) return static_cast<char>('A' + (agentIndex - 36));
    return '*';
}

std::string renderAgents(const Grid& grid, const std::vector<Point>& positions) {
    std::string out = renderGrid(grid);
    std::vector<int> agentsOnCell(static_cast<std::size_t>(grid.numCells()), 0);

    for (std::size_t i = 0; i < positions.size(); ++i) {
        const Point p = positions[i];
        if (!grid.isFree(p.x, p.y)) {
            throw std::invalid_argument("renderAgents: agent " + std::to_string(i) +
                                        " is on an obstacle or outside the grid");
        }
        const auto cell = static_cast<std::size_t>(grid.toId(p));
        // Position of (x, y) inside the string: every row takes width + 1 characters.
        const auto index = static_cast<std::size_t>(p.y * (grid.width() + 1) + p.x);
        ++agentsOnCell[cell];
        out[index] = agentsOnCell[cell] > 1 ? '!' : agentSymbol(static_cast<int>(i));
    }
    return out;
}

std::string renderPath(const Grid& grid, const std::vector<CellId>& path) {
    std::string out = renderGrid(grid);
    auto indexOf = [&](CellId c) {
        if (c < 0 || c >= grid.numCells() || !grid.isFree(c)) {
            throw std::invalid_argument("renderPath: path cell " + std::to_string(c) +
                                        " is an obstacle or outside the grid");
        }
        const Point p = grid.toPoint(c);
        return static_cast<std::size_t>(p.y * (grid.width() + 1) + p.x);
    };
    for (CellId c : path) {
        out[indexOf(c)] = '+';
    }
    if (!path.empty()) {
        out[indexOf(path.front())] = 'S';
        out[indexOf(path.back())] = 'G';
    }
    return out;
}

std::string formatTimedPath(const Grid& grid, const std::vector<CellId>& path) {
    std::string out;
    for (std::size_t t = 0; t < path.size(); ++t) {
        const CellId c = path[t];
        if (c < 0 || c >= grid.numCells()) {
            throw std::invalid_argument("formatTimedPath: cell " + std::to_string(c) +
                                        " is outside the grid");
        }
        const Point p = grid.toPoint(c);
        if (t > 0) out += ' ';
        out += '(' + std::to_string(p.x) + ',' + std::to_string(p.y) + ")@" + std::to_string(t);
    }
    return out;
}

}  // namespace swarmgrid

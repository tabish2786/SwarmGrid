#include "swarmgrid/grid.hpp"

#include <stdexcept>

namespace swarmgrid {

Grid::Grid(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("grid size must be positive");
    }
    width_ = width;
    height_ = height;
    blocked_.assign(width * height, 0);  // every cell starts free
}

bool Grid::inBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool Grid::isFree(int x, int y) const {
    // && short-circuits: if (x, y) is outside, blocked_ is never indexed.
    return inBounds(x, y) && isFree(toId(x, y));
}

bool Grid::isFree(CellId c) const {
    return blocked_[c] == 0;
}

void Grid::setObstacle(int x, int y, bool blocked) {
    if (!inBounds(x, y)) {
        throw std::out_of_range("setObstacle: cell is outside the grid");
    }
    blocked_[toId(x, y)] = blocked ? 1 : 0;
}

CellId Grid::toId(int x, int y) const {
    return y * width_ + x;
}

Point Grid::toPoint(CellId c) const {
    return Point{c % width_, c / width_};
}

void Grid::neighbors(CellId c, std::vector<CellId>& out) const {
    out.clear();
    const Point p = toPoint(c);

    // Fixed order: up, down, left, right.
    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};

    for (int i = 0; i < 4; ++i) {
        const int nx = p.x + dx[i];
        const int ny = p.y + dy[i];
        if (isFree(nx, ny)) {
            out.push_back(toId(nx, ny));
        }
    }
}

int Grid::countObstacles() const {
    int count = 0;
    for (std::uint8_t cell : blocked_) {
        if (cell == 1) {
            ++count;
        }
    }
    return count;
}

}  // namespace swarmgrid

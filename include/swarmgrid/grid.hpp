#pragma once

#include <cstdint>
#include <vector>

namespace swarmgrid {

// Every cell is one int: id = y * width + x. Cheap to store, hash, and index arrays with.
using CellId = int;
constexpr CellId kInvalidCell = -1;

// x = column, y = row, (0,0) = top-left. Same convention as MovingAI files.
struct Point {
    int x = 0;
    int y = 0;
};

inline bool operator==(Point a, Point b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Point a, Point b) { return !(a == b); }

class Grid {
public:
    Grid() = default;

    // All cells free. Throws std::invalid_argument if width <= 0 or height <= 0.
    Grid(int width, int height);

    int width() const { return width_; }
    int height() const { return height_; }
    int numCells() const { return width_ * height_; }

    bool inBounds(int x, int y) const;

    // Out-of-bounds counts as NOT free (no exception). Lets callers skip bounds checks.
    bool isFree(int x, int y) const;
    // Precondition: 0 <= c < numCells(). Not checked in release builds.
    bool isFree(CellId c) const;

    // Throws std::out_of_range if (x, y) is out of bounds.
    void setObstacle(int x, int y, bool blocked);

    // Precondition: inBounds(x, y).
    CellId toId(int x, int y) const;
    CellId toId(Point p) const { return toId(p.x, p.y); }
    // Precondition: 0 <= c < numCells().
    Point toPoint(CellId c) const;

    // Clears `out`, then appends free 4-connected neighbours of c in this FIXED order:
    //   up (y-1), down (y+1), left (x-1), right (x+1).
    // Fixed order => deterministic search. Output param => no allocation in the A* hot loop.
    // Does NOT include c itself (the "wait" action is the search's job, not the grid's).
    void neighbors(CellId c, std::vector<CellId>& out) const;

    int countObstacles() const;

private:
    int width_ = 0;
    int height_ = 0;
    // 1 = obstacle, 0 = free. Interview Q waiting for you: why not std::vector<bool>?
    std::vector<std::uint8_t> blocked_;
};

}  // namespace swarmgrid

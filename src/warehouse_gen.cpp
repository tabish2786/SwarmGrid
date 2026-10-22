#include "swarmgrid/warehouse_gen.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace swarmgrid {

namespace {

// Picks k distinct cells with a partial Fisher-Yates shuffle. `cells` is taken by value so
// every call starts from the same id order, which keeps the result deterministic.
std::vector<Point> pickDistinct(const Grid& grid, std::vector<CellId> cells, int k, Rng& rng) {
    std::vector<Point> picked;
    picked.reserve(static_cast<std::size_t>(k));
    const int last = static_cast<int>(cells.size()) - 1;
    for (int i = 0; i < k; ++i) {
        const int j = rng.uniformInt(i, last);
        std::swap(cells[static_cast<std::size_t>(i)], cells[static_cast<std::size_t>(j)]);
        picked.push_back(grid.toPoint(cells[static_cast<std::size_t>(i)]));
    }
    return picked;
}

}  // namespace

Grid generateWarehouse(const WarehouseParams& params) {
    const WarehouseParams& p = params;
    if (p.blocksX < 1 || p.blocksY < 1 || p.shelfBlockWidth < 1 || p.shelfBlockHeight < 1 ||
        p.aisleWidth < 1 || p.border < 0) {
        throw std::invalid_argument("generateWarehouse: invalid parameters");
    }

    const int width = 2 * p.border + p.blocksX * p.shelfBlockWidth + (p.blocksX - 1) * p.aisleWidth;
    const int height =
        2 * p.border + p.blocksY * p.shelfBlockHeight + (p.blocksY - 1) * p.aisleWidth;
    Grid grid(width, height);

    for (int by = 0; by < p.blocksY; ++by) {
        for (int bx = 0; bx < p.blocksX; ++bx) {
            // Top-left corner of shelf block (bx, by).
            const int left = p.border + bx * (p.shelfBlockWidth + p.aisleWidth);
            const int top = p.border + by * (p.shelfBlockHeight + p.aisleWidth);
            for (int dy = 0; dy < p.shelfBlockHeight; ++dy) {
                for (int dx = 0; dx < p.shelfBlockWidth; ++dx) {
                    grid.setObstacle(left + dx, top + dy, true);
                }
            }
        }
    }
    return grid;
}

int Rng::uniformInt(int lo, int hi) {
    if (lo > hi) {
        throw std::invalid_argument("uniformInt: lo > hi");
    }
    // 64-bit math: hi - lo + 1 can be as large as 2^32, which overflows int.
    const std::uint64_t range = static_cast<std::uint64_t>(static_cast<std::int64_t>(hi) - lo) + 1;
    constexpr std::uint64_t kOutputs = std::uint64_t{1} << 32;  // next() has 2^32 possible values
    if (range == kOutputs) {
        return static_cast<int>(static_cast<std::int64_t>(lo) + next());
    }
    // Rejection sampling. next() % range alone favours small values whenever 2^32 is not a
    // multiple of range, so redraw if the value falls in the incomplete last bucket.
    const std::uint64_t limit = kOutputs - (kOutputs % range);
    std::uint64_t value = next();
    while (value >= limit) {
        value = next();
    }
    return static_cast<int>(static_cast<std::int64_t>(lo) + static_cast<std::int64_t>(value % range));
}

MapfInstance generateRandomInstance(const Grid& grid, int numAgents, std::uint32_t seed) {
    std::vector<CellId> freeCells;
    for (CellId c = 0; c < grid.numCells(); ++c) {
        if (grid.isFree(c)) {
            freeCells.push_back(c);
        }
    }
    if (numAgents <= 0 || static_cast<std::size_t>(numAgents) > freeCells.size()) {
        throw std::invalid_argument("generateRandomInstance: need 1.." +
                                    std::to_string(freeCells.size()) + " agents, got " +
                                    std::to_string(numAgents));
    }

    Rng rng(seed);
    MapfInstance instance;
    instance.grid = grid;
    instance.starts = pickDistinct(grid, freeCells, numAgents, rng);
    instance.goals = pickDistinct(grid, freeCells, numAgents, rng);
    return instance;
}

}  // namespace swarmgrid
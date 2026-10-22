#pragma once

#include <cstdint>
#include <random>

#include "swarmgrid/grid.hpp"
#include "swarmgrid/map_io.hpp"

namespace swarmgrid {

// Layout (blocksX = 2, blocksY = 2, shelfBlockWidth = 3, shelfBlockHeight = 1,
//         aisleWidth = 1, border = 1):
//
//   .........
//   .@@@.@@@.
//   .........
//   .@@@.@@@.
//   .........
//
// width  = 2*border + blocksX*shelfBlockWidth  + (blocksX-1)*aisleWidth
// height = 2*border + blocksY*shelfBlockHeight + (blocksY-1)*aisleWidth
struct WarehouseParams {
    int blocksX = 4;
    int blocksY = 6;
    int shelfBlockWidth = 5;
    int shelfBlockHeight = 2;
    int aisleWidth = 1;
    int border = 2;  // free margin on all 4 sides (stations / parking live here)
};

// Fully deterministic (no randomness). Throws std::invalid_argument if any of
// blocksX, blocksY, shelfBlockWidth, shelfBlockHeight, aisleWidth is < 1 or border < 0.
Grid generateWarehouse(const WarehouseParams& params);

// Portable deterministic RNG.
// std::mt19937's output sequence is fixed by the C++ standard.
// std::uniform_int_distribution and std::shuffle are NOT: MSVC and GCC give different
// results for the same seed. So we never use them -> same seed = same scenario on any machine.
class Rng {
public:
    explicit Rng(std::uint32_t seed) : engine_(seed) {}

    std::uint32_t next() { return static_cast<std::uint32_t>(engine_()); }

    // Uniform integer in [lo, hi] inclusive, built ONLY on next().
    // Must be unbiased: think about why `lo + next() % (hi - lo + 1)` is subtly wrong.
    // Throws std::invalid_argument if lo > hi.
    int uniformInt(int lo, int hi);

private:
    std::mt19937 engine_;
};

// numAgents agents with distinct random free starts and distinct random free goals
// (a start may coincide with another agent's goal; that's legal MAPF).
// Same (grid, numAgents, seed) => identical instance, on every compiler.
// Throws std::invalid_argument if numAgents <= 0 or numAgents > number of free cells.
MapfInstance generateRandomInstance(const Grid& grid, int numAgents, std::uint32_t seed);

}  // namespace swarmgrid

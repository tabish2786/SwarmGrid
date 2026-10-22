// SwarmGrid CLI (M0): load or generate a map, print it, optionally show agents.
//
//   swarmgrid map <file.map> [file.scen] [numAgents]
//   swarmgrid warehouse [numAgents] [seed]
//
// Later milestones add: solve, validate, bench, lifelong.

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include "swarmgrid/ascii.hpp"
#include "swarmgrid/grid.hpp"
#include "swarmgrid/map_io.hpp"
#include "swarmgrid/warehouse_gen.hpp"

namespace {

void printUsage() {
    std::cerr << "usage:\n"
              << "  swarmgrid map <file.map> [file.scen] [numAgents]\n"
              << "  swarmgrid warehouse [numAgents] [seed]\n";
}

void printInstance(const swarmgrid::MapfInstance& inst) {
    std::cout << "starts:\n" << swarmgrid::renderAgents(inst.grid, inst.starts);
    std::cout << "goals:\n" << swarmgrid::renderAgents(inst.grid, inst.goals);
}

int runMap(int argc, char** argv) {
    if (argc < 3) {
        printUsage();
        return 1;
    }
    const swarmgrid::Grid grid = swarmgrid::loadMovingAIMap(argv[2]);
    std::cout << grid.width() << " x " << grid.height() << ", " << grid.countObstacles()
              << " obstacles\n"
              << swarmgrid::renderGrid(grid);
    if (argc >= 4) {
        const auto entries = swarmgrid::loadScenario(argv[3]);
        const int k = argc >= 5 ? std::stoi(argv[4]) : static_cast<int>(entries.size());
        printInstance(swarmgrid::makeInstance(grid, entries, k));
    }
    return 0;
}

int runWarehouse(int argc, char** argv) {
    const int k = argc >= 3 ? std::stoi(argv[2]) : 0;
    const auto seed = static_cast<std::uint32_t>(argc >= 4 ? std::stoul(argv[3]) : 42UL);
    const swarmgrid::Grid grid = swarmgrid::generateWarehouse(swarmgrid::WarehouseParams{});
    std::cout << grid.width() << " x " << grid.height() << ", " << grid.countObstacles()
              << " obstacles\n"
              << swarmgrid::renderGrid(grid);
    if (k > 0) {
        printInstance(swarmgrid::generateRandomInstance(grid, k, seed));
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }
    try {
        const std::string cmd = argv[1];
        if (cmd == "map") return runMap(argc, argv);
        if (cmd == "warehouse") return runWarehouse(argc, argv);
        printUsage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}

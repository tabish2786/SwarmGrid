#pragma once

// CLI commands for M7 (benchmarks) and M8 (lifelong simulation). Kept out of main.cpp, which is
// already long; main.cpp only dispatches to them.
//
//   swarmgrid bench --out results.csv [options]          M7: one CSV row per (instance, solver)
//   swarmgrid lifelong [options]                          M8: one simulation, printed
//   swarmgrid lifelong-bench --out lifelong.csv [options] M8: sweep, one CSV row per run
//
// Run any of them with --help for the options.

int runBench(int argc, char** argv);
int runLifelongCommand(int argc, char** argv);
int runLifelongBench(int argc, char** argv);

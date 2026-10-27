#!/usr/bin/env bash
# Builds docs/swarmgrid-planner.js: the C++ planner compiled to WebAssembly, used by
# docs/control.html. The .wasm is embedded in the .js (SINGLE_FILE), so the page also works when
# opened straight from disk.
#
# Needs Emscripten (https://emscripten.org/docs/getting_started/downloads.html), tested with 3.1.74:
#   source /path/to/emsdk/emsdk_env.sh
#   ./web/build_wasm.sh            # run from the repo root
set -euo pipefail
cd "$(dirname "$0")/.."

EXPORTS='_sg_width,_sg_height,_sg_is_free,_sg_clear_agents,_sg_add_agent,_sg_random_agents,'\
'_sg_agent_count,_sg_agent_start,_sg_agent_goal,_sg_plan,_sg_path_length,_sg_path_cell,'\
'_sg_failed_agent,_sg_attempts,_sg_expanded,_sg_last_error,_sg_plan_with_limits,_sg_ct_nodes,'\
'_sg_limit_reached'

em++ -std=c++17 -O2 -fexceptions -Wall -Wextra -Iinclude \
  src/grid.cpp src/heuristic.cpp src/astar.cpp src/constraints.cpp src/space_time_astar.cpp \
  src/prioritized.cpp src/conflicts.cpp src/cbs.cpp src/warehouse_gen.cpp web/wasm_api.cpp \
  -sMODULARIZE=1 -sEXPORT_NAME=createSwarmGridPlanner -sSINGLE_FILE=1 \
  -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web,node -sFILESYSTEM=0 \
  -sEXPORTED_FUNCTIONS="$EXPORTS" -sEXPORTED_RUNTIME_METHODS=ccall,UTF8ToString \
  -o docs/swarmgrid-planner.js

echo "wrote docs/swarmgrid-planner.js ($(wc -c < docs/swarmgrid-planner.js) bytes)"

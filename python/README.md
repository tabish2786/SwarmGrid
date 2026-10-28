# swarmgrid (Python)

Collision-free paths for warehouse robot fleets, from Python. A thin layer over the
[SwarmGrid](https://github.com/tabish2786/SwarmGrid) C++17 core: optimal Conflict-Based Search
(with MDD conflict prioritization, bypass, and an optional machine-learned conflict ranker),
bounded-suboptimal ECBS, prioritized planning, optimal (Hungarian) order assignment and a lifelong
warehouse simulator with rolling-horizon replanning.

```python
import swarmgrid as sg

grid = sg.warehouse()                                  # 27 x 21 generated warehouse
starts, goals = sg.random_agents(grid, 10, seed=42)
plan = sg.solve(grid, starts, goals)                   # optimal CBS
print(plan.sum_of_costs, plan.makespan)                # 208 38
assert sg.validate(grid, starts, goals, plan.paths).ok
```

## Install

You need Python 3.9+ and a C++17 compiler; pip compiles the C++ part during the install.

```bash
pip install "git+https://github.com/tabish2786/SwarmGrid.git"   # latest from GitHub
# or, from a clone of the repository:
pip install .
```

- **Linux / macOS:** GCC or Clang (`build-essential` on Ubuntu, Xcode command-line tools on macOS).
  Google Colab already has everything: `!pip install git+https://github.com/tabish2786/SwarmGrid.git`.
- **Windows:** the Python from python.org needs the Microsoft C++ compiler: install "Build Tools for
  Visual Studio" with the "Desktop development with C++" workload. (A MinGW compiler works only with
  a MinGW-built Python, such as MSYS2's.)

The distribution is called `swarmgrid-mapf` (the name `swarmgrid` on PyPI belongs to an unrelated
project); you still `import swarmgrid`.

## Concepts

- **Points** are `(x, y)` tuples: `x` is the column, `y` the row, `(0, 0)` the top-left cell.
- **Time** is discrete. Each step a robot moves to one of its 4 neighbours or waits.
  `plan.paths[i][t]` is robot i's cell at time t; after its path ends it stays on its goal.
- A plan is **valid** if no two robots share a cell at the same time (vertex conflict), swap cells
  in one step (edge conflict), or drive through a robot already parked on its goal (target conflict).
- The **sum of costs** (every robot's arrival time, added up) is what the planners minimise; the
  **makespan** is when the last robot arrives.

## API

| Function | What it does |
|---|---|
| `warehouse(blocks_x=4, blocks_y=6, shelf_width=5, shelf_height=2, aisle=1, border=2)` | Generated warehouse. Defaults: 27 x 21. `warehouse(8, 8, 10, 2, 2, 3)`: the 100 x 36 benchmark map. |
| `grid_from_rows(rows)` | Grid from strings (`.` free, `@` obstacle). |
| `load_map(path)`, `load_scenario(grid, path, agents)` | MovingAI `.map` files and the first `agents` entries of a `.scen` file. |
| `random_agents(grid, agents, seed)` | Distinct random starts and goals; same seed = same robots everywhere. |
| `solve(grid, starts, goals, method="cbs", ...)` | Plan all robots. Returns a `Plan`. |
| `validate(grid, starts, goals, paths)` | Independent re-check of any plan. Returns a `Validation` (truthy if valid). |
| `shortest_path(grid, start, goal)` | One robot alone (A* with exact distances). |
| `assign(grid, robots, orders, method="hungarian")` | Robots to orders: optimal Hungarian or greedy. |
| `simulate_lifelong(grid, robots=20, steps=500, ...)` | Warehouse with a stream of orders. Returns `LifelongStats`. |
| `conflict_samples(grid, starts, goals, ...)` | Training data for learning to pick CBS conflicts. |
| `render(grid, positions=())` | The map as text, robots as `0-9a-zA-Z`. |
| `Grid` | `width`, `height`, `is_free(x, y)`, `set_obstacle(x, y, blocked=True)`, `rows()`, `free_cells()`, `copy()`. |

Every function has a docstring: `help(sg.solve)`.

### `solve`

```python
plan = sg.solve(grid, starts, goals,
                method="cbs",                 # "cbs" | "ecbs" | "prioritized" | "independent"
                time_limit=None,              # seconds; None = no limit (fully deterministic)
                max_nodes=100_000,            # constraint-tree nodes before giving up
                conflict_selection="cardinal",# CBS: "first" | "cardinal" | "learned" | "oracle"
                bypass=True,                  # CBS: bypass equally cheap children
                weight=1.2,                   # ECBS: sum of costs <= weight * optimal
                window=None,                  # resolve conflicts only up to this timestep
                model=None)                   # ranker for "learned": file path or 18 integer weights
```

| `method` | Guarantee | Use it when |
|---|---|---|
| `"cbs"` | optimal sum of costs | up to a few dozen robots; you want the best plan |
| `"ecbs"` | sum of costs <= `weight` x optimal | many robots (100+ on the benchmark map) |
| `"prioritized"` | collision-free, not optimal, can fail on solvable instances | speed matters most |
| `"independent"` | none (robots ignore each other) | a baseline that shows the collisions |

The `Plan` has `solved`, `paths`, `sum_of_costs`, `makespan`, `lower_bound` (no valid plan is
cheaper), `nodes_expanded`, `seconds`, `limit_reached`, and `stats` with everything else the solver
reported. `bool(plan)` is `plan.solved`.

### CBS conflict selection

CBS repeatedly splits a conflict between two robots into two sub-problems. Which conflict it
splits does not change the answer (always optimal), only how much searching it takes:

- `"first"`: the earliest conflict (textbook CBS).
- `"cardinal"` (default): conflicts that must make both robots' paths longer go first, found with
  multi-valued decision diagrams (Boyarski et al., 2015).
- `"learned"`: a linear model scores 18 features of every conflict and CBS splits the best one. It
  was trained to imitate `"oracle"` (see below). Pass your own weights with `model=`.
- `"oracle"`: splits every conflict for real and keeps the one that raises the lower bound most.
  Small trees, but every node costs two searches per conflict.

### Learning your own ranker

`conflict_samples` runs CBS and records, for each conflict of its first `nodes` nodes, the 18
features (`sg.conflict_feature_names()`) and what the oracle measured. Fit any ranking model, turn
it into 18 integer weights, and pass them back:

```python
data = sg.conflict_samples(grid, starts, goals, nodes=50, conflicts=20)
# ... train a linear ranker on (data.features, data.min_increase, data.sum_increase) ...
plan = sg.solve(grid, starts, goals, conflict_selection="learned", model=my_weights)
```

[`examples/train_ranker.py`](https://github.com/tabish2786/SwarmGrid/blob/main/python/examples/train_ranker.py) does this end to end with numpy in about 40
seconds and compares the result with the shipped model.

### Lifelong simulation

```python
stats = sg.simulate_lifelong(grid, robots=30, steps=500,
                             window=10, replan_every=5,   # RHCR: plan 10 steps, replan every 5
                             solver="ecbs", assignment="hungarian",
                             breakdown_rate=0.001, seed=1)
print(stats.delivered, stats.throughput, stats.collisions)
```

Robots fetch orders placed next to shelves and drop them at stations on the map's outer ring. A
robot that makes no progress for `stuck_steps` takes a short detour (deadlock recovery); broken
robots block their cell until repaired. `record_positions=True` returns every robot's position at
every step, for animations.

## Examples

| File | Shows |
|---|---|
| [`examples/quickstart.py`](https://github.com/tabish2786/SwarmGrid/blob/main/python/examples/quickstart.py) | plan, validate, see the collisions of uncoordinated robots |
| [`examples/compare_planners.py`](https://github.com/tabish2786/SwarmGrid/blob/main/python/examples/compare_planners.py) | CBS, learned CBS, ECBS and prioritized planning on 10-50 robots |
| [`examples/lifelong_demo.py`](https://github.com/tabish2786/SwarmGrid/blob/main/python/examples/lifelong_demo.py) | 500 steps of a warehouse, greedy vs Hungarian assignment |
| [`examples/train_ranker.py`](https://github.com/tabish2786/SwarmGrid/blob/main/python/examples/train_ranker.py) | train a conflict ranker in Python and use it in CBS |

## Good to know

- **Threads:** solves release the GIL, so `concurrent.futures.ThreadPoolExecutor` runs several in
  parallel. A running solve cannot be interrupted with Ctrl+C; use `time_limit`.
- **Determinism:** without a time limit, the same input gives the same plan on every platform,
  in the C++ tool and in the web demo (the learned ranker uses integer arithmetic for this).
- **Limits:** 4-connected grids, unit-time moves, no robot size, turning or delays. CBS gets slow
  past a few dozen robots in tight spaces; use ECBS there.

## Development

```bash
pip install .                                      # build + install
python -m unittest discover -s tests/python -v     # run from the repository root
```

The C++ library, its own test suite and the benchmarks live in the main repository; see its
[README](https://github.com/tabish2786/SwarmGrid#readme).

License: MIT.

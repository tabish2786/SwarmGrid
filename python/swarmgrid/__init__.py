"""SwarmGrid: collision-free paths for warehouse robot fleets (multi-agent path finding).

A thin Python layer over the SwarmGrid C++17 core. Everything heavy runs in C++, and long searches
release the GIL, so several solves can run in parallel threads.

Points are ``(x, y)`` tuples: ``x`` is the column, ``y`` the row, ``(0, 0)`` the top-left cell.
Robots move to one of the 4 neighbouring cells or wait, one step per timestep, and stay on their
goal once they arrive. A plan is valid if no two robots share a cell at the same time, swap cells,
or drive through a robot that has already parked.

Quick start::

    import swarmgrid as sg

    grid = sg.warehouse()                          # 27 x 21 generated warehouse
    starts, goals = sg.random_agents(grid, 10, seed=42)
    plan = sg.solve(grid, starts, goals)           # optimal CBS
    print(plan.sum_of_costs, plan.makespan)        # 208 38
    assert sg.validate(grid, starts, goals, plan.paths).ok

See https://github.com/tabish2786/SwarmGrid/blob/main/python/README.md for the full guide.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple, Union

from . import _core
from ._core import Grid

__version__ = "0.2.0"

__all__ = [
    "Grid", "Point", "Path", "Plan", "Validation", "Conflict", "Assignment", "LifelongStats", "ConflictSamples",
    "warehouse", "grid_from_rows", "load_map", "load_scenario", "random_agents", "shortest_path", "render",
    "solve", "validate", "assign", "simulate_lifelong", "conflict_samples", "conflict_feature_names",
    "builtin_ranker_weights", "load_ranker", "__version__",
]

def _existing(path: Union[str, "os.PathLike[str]"]) -> str:
    path = os.fspath(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    return path


Point = Tuple[int, int]
Path = List[Point]
RankerModel = Union[str, "os.PathLike[str]", Sequence[int], None]


# ----------------------------------------------------------------------------- maps and agents

def warehouse(blocks_x: int = 4, blocks_y: int = 6, shelf_width: int = 5, shelf_height: int = 2,
              aisle: int = 1, border: int = 2) -> Grid:
    """A generated warehouse: ``blocks_x`` x ``blocks_y`` blocks of shelves separated by aisles.

    The defaults give the 27 x 21 map (240 shelf cells) used throughout SwarmGrid.
    ``warehouse(8, 8, 10, 2, 2, 3)`` gives the 100 x 36 map of the benchmarks.
    """
    return _core.warehouse(blocks_x, blocks_y, shelf_width, shelf_height, aisle, border)


def grid_from_rows(rows: Sequence[str]) -> Grid:
    """A grid from strings, one per row. Free: ``.`` ``G`` ``S``; obstacles: ``@`` ``O`` ``T`` ``W``.

    >>> grid_from_rows(["....", ".@@.", "...."]).count_obstacles()
    2
    """
    return _core.grid_from_rows(list(rows))


def load_map(path: Union[str, "os.PathLike[str]"]) -> Grid:
    """Load a MovingAI ``.map`` file (https://movingai.com/benchmarks/mapf.html)."""
    return _core.load_map(_existing(path))


def load_scenario(grid: Grid, path: Union[str, "os.PathLike[str]"], agents: int) -> Tuple[List[Point], List[Point]]:
    """The first ``agents`` start/goal pairs of a MovingAI ``.scen`` file, as ``(starts, goals)``."""
    return _core.scenario_agents(grid, _existing(path), agents)


def random_agents(grid: Grid, agents: int, seed: int = 0) -> Tuple[List[Point], List[Point]]:
    """``agents`` random distinct starts and distinct goals on free cells, as ``(starts, goals)``.

    Deterministic: the same seed gives the same robots on every platform, and the same as the
    ``swarmgrid`` command-line tool and the web demo.
    """
    return _core.random_agents(grid, agents, seed)


def shortest_path(grid: Grid, start: Point, goal: Point) -> Optional[Path]:
    """Shortest path of one robot ignoring all others (A* with exact BFS distances), or ``None``."""
    return _core.shortest_path(grid, tuple(start), tuple(goal))


def render(grid: Grid, positions: Sequence[Point] = ()) -> str:
    """The grid as text (``.`` free, ``@`` obstacle), with robots drawn as 0-9, a-z, A-Z."""
    return _core.render(grid, [tuple(p) for p in positions])


# ----------------------------------------------------------------------------- planning

@dataclass
class Plan:
    """Result of :func:`solve`.

    ``paths[i][t]`` is robot i's cell at time t; after its path ends a robot stays on its goal.
    ``lower_bound``: no valid plan has a smaller sum of costs (the optimum itself for a solved CBS;
    for ECBS ``sum_of_costs <= weight * lower_bound``; 0 when unknown).
    """

    method: str
    solved: bool
    paths: List[Path]
    sum_of_costs: int
    makespan: int
    lower_bound: int
    seconds: float
    limit_reached: bool = False
    nodes_expanded: int = 0
    stats: Dict[str, Union[int, float]] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.solved


def _ranker_weights(model: RankerModel) -> Optional[List[int]]:
    if model is None:
        return None
    if isinstance(model, (str, os.PathLike)):
        return load_ranker(model)
    weights = [int(w) for w in model]
    names = conflict_feature_names()
    if len(weights) != len(names):
        raise ValueError(f"a ranker needs {len(names)} integer weights, got {len(weights)}")
    return weights


def solve(grid: Grid, starts: Sequence[Point], goals: Sequence[Point], method: str = "cbs", *,
          time_limit: Optional[float] = None, max_nodes: int = 100_000, conflict_selection: str = "cardinal",
          bypass: bool = True, weight: float = 1.2, window: Optional[int] = None,
          model: RankerModel = None) -> Plan:
    """Plan collision-free paths for every robot.

    Args:
        grid: the map.
        starts, goals: one ``(x, y)`` per robot; starts distinct, goals distinct, all on free cells.
        method: ``"cbs"`` (optimal sum of costs), ``"ecbs"`` (sum of costs <= ``weight`` x optimal,
            scales to many more robots), ``"prioritized"`` (fast, not optimal, can fail on solvable
            instances) or ``"independent"`` (each robot's own shortest path; may collide).
        time_limit: seconds before CBS/ECBS give up (``None`` = no limit). Results are deterministic
            only without a time limit.
        max_nodes: constraint-tree nodes CBS/ECBS may expand before giving up.
        conflict_selection: CBS only. ``"first"`` (plain CBS), ``"cardinal"`` (MDD-based
            prioritization, the default), ``"learned"`` (ML ranking model, see ``model``) or
            ``"oracle"`` (split every conflict for real; small trees, slow nodes). All are optimal.
        bypass: CBS only. Take a child's equally cheap path instead of branching when possible.
        weight: ECBS suboptimality bound, >= 1.
        window: rolling horizon. Only conflicts up to this timestep are resolved (CBS, ECBS,
            prioritized). ``None`` = whole paths.
        model: ranker for ``conflict_selection="learned"``: a model file path, a list of integer
            weights (see :func:`conflict_feature_names`), or ``None`` for the model shipped with
            SwarmGrid.

    Returns:
        A :class:`Plan`. ``plan.solved`` is False if the limit was hit or no plan exists.

    Raises:
        ValueError: bad robots (outside the grid, on an obstacle, shared start or goal) or options.
    """
    starts = [tuple(p) for p in starts]
    goals = [tuple(p) for p in goals]
    win = -1 if window is None else int(window)
    limit = 0.0 if time_limit is None else float(time_limit)
    if method == "cbs":
        r = _core.solve_cbs(grid, starts, goals, int(max_nodes), limit, conflict_selection, bool(bypass), win,
                            _ranker_weights(model))
    elif method == "ecbs":
        r = _core.solve_ecbs(grid, starts, goals, float(weight), int(max_nodes), limit, win)
    elif method in ("prioritized", "independent"):
        r = _core.solve_prioritized(grid, starts, goals, method == "independent", 10, win)
    else:
        raise ValueError(f"method must be 'cbs', 'ecbs', 'prioritized' or 'independent', got {method!r}")
    main = {"solved", "paths", "sum_of_costs", "makespan", "lower_bound", "seconds", "limit_reached", "nodes_expanded"}
    return Plan(method=method, solved=r["solved"], paths=r["paths"], sum_of_costs=r["sum_of_costs"],
                makespan=r["makespan"], lower_bound=r.get("lower_bound", 0), seconds=r.get("seconds", 0.0),
                limit_reached=r.get("limit_reached", False), nodes_expanded=r.get("nodes_expanded", 0),
                stats={k: v for k, v in r.items() if k not in main})


@dataclass
class Conflict:
    """Two robots on one ``cell`` at time ``t`` (``vertex``; ``target`` if one has parked there), or
    swapping ``cell`` and ``to`` between t-1 and t (``edge``)."""

    type: str
    agents: Tuple[int, int]
    cell: Point
    to: Optional[Point]
    t: int


@dataclass
class Validation:
    """Result of :func:`validate`: ``ok`` if there are no errors and no conflicts."""

    ok: bool
    errors: List[str]
    conflicts: List[Conflict]
    sum_of_costs: int
    makespan: int

    def __bool__(self) -> bool:
        return self.ok


def validate(grid: Grid, starts: Sequence[Point], goals: Sequence[Point], paths: Sequence[Sequence[Point]]) -> Validation:
    """Re-check a plan without trusting any solver: right starts and goals, one-cell moves onto free
    cells, and no vertex, edge (swap) or target conflicts."""
    r = _core.validate(grid, [tuple(p) for p in starts], [tuple(p) for p in goals],
                       [[tuple(p) for p in path] for path in paths])
    return Validation(ok=r["ok"], errors=list(r["errors"]), conflicts=[Conflict(**c) for c in r["conflicts"]],
                      sum_of_costs=r["sum_of_costs"], makespan=r["makespan"])


# ----------------------------------------------------------------------------- orders

@dataclass(frozen=True)
class Assignment:
    """Robot ``robot`` takes order ``order``; ``distance`` is the true grid distance (around shelves)."""

    robot: int
    order: int
    distance: int


def assign(grid: Grid, robots: Sequence[Point], orders: Sequence[Point], method: str = "hungarian") -> List[Assignment]:
    """Give orders to robots. ``"hungarian"``: as many reachable pairs as possible with the least
    total distance (optimal). ``"greedy"``: repeatedly the closest free pair (fast, not optimal).
    Indices refer to the input lists; robots or orders left over are not listed."""
    if method not in ("hungarian", "greedy"):
        raise ValueError(f"method must be 'hungarian' or 'greedy', got {method!r}")
    pairs = _core.assign(grid, [tuple(p) for p in robots], [tuple(p) for p in orders], method == "hungarian")
    return [Assignment(*p) for p in pairs]


@dataclass
class LifelongStats:
    """Result of :func:`simulate_lifelong`. ``throughput`` = orders delivered per timestep.
    ``positions[t][i]`` (only with ``record_positions=True``) is robot i's cell at time t."""

    steps: int
    delivered: int
    throughput: float
    replans: int
    planner_failures: int
    all_wait: int
    deadlocks: int
    escapes: int
    breakdowns: int
    collisions: int
    assigned_distance: int
    mean_plan_seconds: float
    max_plan_seconds: float
    positions: List[List[Point]] = field(default_factory=list, repr=False)


def simulate_lifelong(grid: Grid, robots: int = 20, steps: int = 500, *, window: Optional[int] = 10,
                      replan_every: int = 5, solver: str = "ecbs", weight: float = 1.2,
                      assignment: str = "hungarian", backlog: int = 0, breakdown_rate: float = 0.0,
                      repair_steps: int = 20, stuck_steps: int = 40, planner_max_nodes: int = 2000,
                      planner_time_limit: Optional[float] = None, seed: int = 1,
                      record_positions: bool = False) -> LifelongStats:
    """Run a warehouse with a stream of orders (rolling-horizon replanning, RHCR).

    Orders appear next to shelves; a free robot is assigned one, drives to it, then to a drop-off
    station on the map's outer ring. Every ``replan_every`` steps the fleet is replanned, resolving
    conflicts only for the next ``window`` steps (``None`` = whole paths). A robot that makes no
    progress for ``stuck_steps`` takes a short detour (deadlock recovery). With ``breakdown_rate``
    > 0, robots break down at random and block their cell for ``repair_steps`` steps.

    ``solver``: ``"ecbs"``, ``"cbs"`` or ``"prioritized"``; ``assignment``: ``"hungarian"`` or
    ``"greedy"``. Deterministic for a fixed seed when ``planner_time_limit`` is ``None``.
    """
    r = _core.lifelong(grid, robots, steps, -1 if window is None else window, replan_every, solver, weight,
                       assignment, backlog, breakdown_rate, repair_steps, stuck_steps, planner_max_nodes,
                       0.0 if planner_time_limit is None else planner_time_limit, seed, record_positions)
    return LifelongStats(**r)


# ----------------------------------------------------------------------------- learning to split conflicts

def conflict_feature_names() -> List[str]:
    """Names of the 18 integer features CBS computes for every conflict (the ranker's inputs)."""
    return _core.conflict_feature_names()


def builtin_ranker_weights() -> List[int]:
    """Fixed-point weights of the conflict ranker shipped with SwarmGrid (same order as the names)."""
    return _core.builtin_ranker_weights()


def load_ranker(path: Union[str, "os.PathLike[str]"]) -> List[int]:
    """Weights from a model file (``feature_name weight`` per line, as written by
    ``scripts/train_conflict_ranker.py``)."""
    return _core.load_ranker_weights(_existing(path))


@dataclass
class ConflictSamples:
    """Training data from :func:`conflict_samples`. Row k: ``features[k]`` of one conflict at CBS node
    ``node[k]``, and the oracle's measurement when that conflict was split for real:
    ``min_increase[k]`` (how much the cheaper child costs more than the node; higher = better
    split) and ``sum_increase[k]`` (both children together). Rank conflicts within a node."""

    feature_names: List[str]
    features: List[List[int]]
    node: List[int]
    conflict_index: List[int]
    min_increase: List[int]
    sum_increase: List[int]

    def __len__(self) -> int:
        return len(self.features)


def conflict_samples(grid: Grid, starts: Sequence[Point], goals: Sequence[Point], *, collect: str = "cardinal",
                     nodes: int = 50, conflicts: int = 20, model: RankerModel = None) -> ConflictSamples:
    """Run CBS on one instance and record oracle-labelled conflict features at its first ``nodes``
    nodes (up to ``conflicts`` per node), e.g. to train your own ranker with scikit-learn or PyTorch.

    ``collect`` picks the conflict selection that drives the search while recording (``"learned"``
    with your ``model`` gives DAgger-style data on the model's own nodes)."""
    r = _core.conflict_samples(grid, [tuple(p) for p in starts], [tuple(p) for p in goals], collect, nodes,
                               conflicts, _ranker_weights(model))
    return ConflictSamples(feature_names=conflict_feature_names(), features=r["features"], node=r["node"],
                           conflict_index=r["conflict_index"], min_increase=r["min_increase"],
                           sum_increase=r["sum_increase"])

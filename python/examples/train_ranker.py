"""Train your own CBS conflict ranker in Python and compare it with the shipped one.

    pip install numpy
    python train_ranker.py

1. Run CBS on training instances and record, for every conflict of the first nodes, 18 integer
   features and what happened when that conflict was split for real (the "oracle").
2. Fit a linear ranking model (pairwise logistic regression) with numpy.
3. Turn the weights into fixed-point integers and let CBS use them on unseen instances.
"""
import numpy as np

import swarmgrid as sg

grid = sg.warehouse()
rng = np.random.default_rng(0)

# 1. Training data: 15-20 robots on the small warehouse, seeds 2001-2020.
x_rows, keys, groups = [], [], []
for seed in range(2001, 2021):
    starts, goals = sg.random_agents(grid, 15 + seed % 6, seed=seed)
    data = sg.conflict_samples(grid, starts, goals, nodes=40, conflicts=15)
    x_rows += data.features
    keys += [lo * 10000 + total for lo, total in zip(data.min_increase, data.sum_increase)]
    groups += [(seed, n) for n in data.node]
x, keys = np.array(x_rows, dtype=float), np.array(keys)
print(f"{len(x)} conflicts from {len(set(groups))} CBS nodes")

# 2. Pairs (better, worse) within each node -> logistic regression on x_better - x_worse.
std = x.std(axis=0)
std[std == 0] = 1
diffs = []
by_node = {}
for i, g in enumerate(groups):
    by_node.setdefault(g, []).append(i)
for rows in by_node.values():
    rows = np.array(rows)
    better, worse = np.nonzero(keys[rows][:, None] > keys[rows][None, :])
    diffs.append((x[rows[better]] - x[rows[worse]]) / std)
d = np.vstack(diffs)
w = np.zeros(x.shape[1])
for _ in range(30):                                    # Newton's method
    p = 1 / (1 + np.exp(np.clip(d @ w, -50, 50)))
    grad = -(d * p[:, None]).mean(axis=0) + 1e-3 * w
    hess = (d * (p * (1 - p))[:, None]).T @ d / len(d) + 1e-3 * np.eye(len(w))
    w -= np.linalg.solve(hess, grad)
weights = [int(v) for v in np.rint(w / std * 2**16)]
print("largest weights:", sorted(zip(sg.conflict_feature_names(), weights), key=lambda t: -abs(t[1]))[:4])

# 3. Compare on unseen instances (tree nodes CBS needs; every variant finds the same optimum).
totals = {"cardinal (M5)": 0, "shipped model": 0, "your model": 0}
for seed in range(1, 11):
    starts, goals = sg.random_agents(grid, 20, seed=seed)
    runs = {
        "cardinal (M5)": sg.solve(grid, starts, goals, time_limit=5),
        "shipped model": sg.solve(grid, starts, goals, conflict_selection="learned", time_limit=5),
        "your model": sg.solve(grid, starts, goals, conflict_selection="learned", model=weights, time_limit=5),
    }
    if all(r.solved for r in runs.values()):
        for name, r in runs.items():
            totals[name] += r.nodes_expanded
print("total CT nodes on unseen instances solved by all three:", totals)

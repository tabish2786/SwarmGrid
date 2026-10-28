#!/usr/bin/env python3
"""Train the CBS conflict ranker from `swarmgrid ml-data` CSV files.

    ./build/swarmgrid ml-data --out ml-large.csv --generated large --agents 40,50,60 --seeds 1001-1060
    ./build/swarmgrid ml-data --out ml-small.csv --generated small --agents 15,20,25 --seeds 1001-1060
    python3 scripts/train_conflict_ranker.py ml-large.csv ml-small.csv

Each CSV row is one conflict of one CBS node: integer features (see ConflictFeature in
include/swarmgrid/conflict_ranking.hpp) and the oracle's measurement when that conflict was split
for real (how much the cheaper child, and both children, cost more than the node). Conflicts of
the same node are ranked against each other: a conflict is better if its min_increase is larger
(it raises the lower bound more), then if its sum_increase is larger.

Model: a linear score w . x, trained with pairwise logistic regression (RankNet with a linear
model): for every pair (better, worse) of one node, minimise log(1 + exp(-w . (x_better - x_worse)))
plus a small L2 penalty, with Newton's method (18 weights, so the Hessian is tiny). Features are
divided by their standard deviation while training; the weights are then turned into fixed-point
integers (w / std * 2^16) so that C++ can score with exact integer arithmetic.

Instances whose seed is a multiple of 5 are held out and only used to report how well the model
imitates the oracle, next to two fixed rules (earliest conflict, and the M5 cardinal rule).

Writes models/conflict_ranker.txt (readable, loadable with --model) and
src/conflict_ranker_model.inc (compiled into the library as the default model). Needs numpy only.
"""
import argparse
import csv
import os
from collections import defaultdict

import numpy as np

FEATURES = [
    "is_vertex", "is_edge", "is_target", "is_cardinal", "is_semi_cardinal", "time",
    "mdd_width_min", "mdd_width_max", "agent_conflicts_min", "agent_conflicts_max",
    "path_cost_min", "path_cost_max", "delay_min", "delay_max", "constraints_min",
    "constraints_max", "goal_dist_min", "goal_dist_max",
]
SCALE = 1 << 16


def load(paths):
    """Returns a list of groups: (instance key, X (m x F int64), label key (m), conflict index (m))."""
    groups = defaultdict(list)
    for file_index, path in enumerate(paths):
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                instance = (file_index, row["map"], int(row["agents"]), int(row["seed"]))
                groups[(instance, int(row["node"]))].append(row)
    out = []
    for (instance, _node), rows in sorted(groups.items()):
        x = np.array([[int(r[name]) for name in FEATURES] for r in rows], dtype=np.int64)
        # Lexicographic (min_increase, sum_increase) as one number; increases are <= 2000.
        key = np.array([int(r["min_increase"]) * 10000 + int(r["sum_increase"]) for r in rows], dtype=np.int64)
        idx = np.array([int(r["conflict_index"]) for r in rows], dtype=np.int64)
        out.append((instance, x, key, idx))
    return out


def pairs(groups, rng, max_pairs):
    """x_better - x_worse for every (better, worse) pair of each node, at most max_pairs per node."""
    diffs = []
    for _inst, x, key, _idx in groups:
        better, worse = np.nonzero(key[:, None] > key[None, :])
        if len(better) > max_pairs:
            keep = rng.choice(len(better), size=max_pairs, replace=False)
            better, worse = better[keep], worse[keep]
        if len(better):
            diffs.append(x[better] - x[worse])
    return np.vstack(diffs).astype(np.float64)


def train(d, l2, iterations=60):
    """Newton's method on mean pairwise logistic loss + l2/2 |w|^2."""
    w = np.zeros(d.shape[1])
    for _ in range(iterations):
        z = d @ w
        p = 1.0 / (1.0 + np.exp(np.clip(z, -50, 50)))           # sigmoid(-z)
        grad = -(d * p[:, None]).mean(axis=0) + l2 * w
        s = p * (1.0 - p)
        hess = (d * s[:, None]).T @ d / len(d) + l2 * np.eye(d.shape[1])
        step = np.linalg.solve(hess, grad)
        w -= step
        if np.abs(step).max() < 1e-10:
            break
    return w


def evaluate(groups, choose):
    """choose(x, key_unused, idx) -> chosen row. Returns metrics over informative groups."""
    top1 = regret = pair_ok = pair_n = 0.0
    informative = 0
    for _inst, x, key, idx in groups:
        if len(set(key.tolist())) < 2:
            continue  # all conflicts equally good: nothing to learn or test here
        informative += 1
        c = choose(x, idx)
        top1 += key[c] == key.max()
        regret += (key.max() // 10000) - (key[c] // 10000)
    return {"groups": informative, "top1": top1 / max(informative, 1), "regret": regret / max(informative, 1)}


def pairwise_accuracy(groups, score):
    """Share of (better, worse) pairs the score orders correctly; ties count half."""
    ok = n = 0.0
    for _inst, x, key, _idx in groups:
        s = score(x)
        better, worse = np.nonzero(key[:, None] > key[None, :])
        n += len(better)
        ok += np.sum(s[better] > s[worse]) + 0.5 * np.sum(s[better] == s[worse])
    return ok / max(n, 1)


def argmax_first(values, idx):
    """Highest value; ties go to the earliest conflict (smallest index), as in the C++ code."""
    best = None
    for r in range(len(values)):
        if best is None or values[r] > values[best] or (values[r] == values[best] and idx[r] < idx[best]):
            best = r
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--l2", type=float, default=1e-3)
    ap.add_argument("--max-pairs-per-node", type=int, default=200)
    ap.add_argument("--seed", type=int, default=7)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--model-out", default=os.path.join(root, "models", "conflict_ranker.txt"))
    ap.add_argument("--inc-out", default=os.path.join(root, "src", "conflict_ranker_model.inc"))
    args = ap.parse_args()

    groups = load(args.csv)
    held_out = [g for g in groups if g[0][3] % 5 == 0]
    training = [g for g in groups if g[0][3] % 5 != 0]
    rng = np.random.default_rng(args.seed)
    all_x = np.vstack([g[1] for g in training]).astype(np.float64)
    std = all_x.std(axis=0)
    std[std == 0] = 1.0
    d = pairs(training, rng, args.max_pairs_per_node) / std
    w = train(d, args.l2)
    weights = np.rint(w / std * SCALE).astype(np.int64)

    def int_score(x):
        return x @ weights

    rules = {
        "earliest conflict (M4)": lambda x, idx: argmax_first(np.zeros(len(idx)), idx),
        "cardinal rule (M5)": lambda x, idx: argmax_first(2 * x[:, 3] + x[:, 4], idx),
        "learned (this model)": lambda x, idx: argmax_first(int_score(x), idx),
    }
    n_train_inst = len({g[0] for g in training})
    n_test_inst = len({g[0] for g in held_out})
    lines = [
        f"trained on {sum(len(g[2]) for g in training)} conflicts from {len(training)} nodes of "
        f"{n_train_inst} instances ({len(d)} pairs); held out {sum(len(g[2]) for g in held_out)} "
        f"conflicts from {len(held_out)} nodes of {n_test_inst} instances (seed % 5 == 0)",
        f"held-out pairwise accuracy of the learned model: {pairwise_accuracy(held_out, int_score):.3f}",
        "held-out nodes where conflicts differ: picks the oracle's best conflict / mean lower-bound "
        "increase lost vs the oracle:",
    ]
    for name, rule in rules.items():
        m = evaluate(held_out, rule)
        lines.append(f"  {name:24s} {m['top1']:.3f} / {m['regret']:.3f}   ({m['groups']} nodes)")
    print("\n".join(lines))
    print("weights (fixed point, x 2^16 / feature std):")
    for name, v, raw in zip(FEATURES, weights, w):
        print(f"  {name:22s} {int(v):>12d}   (standardised {raw:+.3f})")

    if os.path.dirname(args.model_out):
        os.makedirs(os.path.dirname(args.model_out), exist_ok=True)
    with open(args.model_out, "w", newline="\n") as f:
        f.write("# SwarmGrid conflict ranker: score = sum of weight * feature (fixed point)\n")
        f.write("# Written by scripts/train_conflict_ranker.py from: "
                + ", ".join(os.path.basename(p) for p in args.csv) + "\n")
        for line in lines:
            f.write(f"# {line}\n")
        for name, v in zip(FEATURES, weights):
            f.write(f"{name} {int(v)}\n")
    with open(args.inc_out, "w", newline="\n") as f:
        f.write("// Generated by scripts/train_conflict_ranker.py (see models/conflict_ranker.txt for the\n"
                "// training data and held-out metrics). Do not edit by hand.\n"
                "constexpr std::array<std::int64_t, kConflictFeatures> kBuiltInConflictWeights = {\n    ")
        f.write(", ".join(str(int(v)) for v in weights))
        f.write(",\n};\n")
    print(f"wrote {args.model_out} and {args.inc_out}")


if __name__ == "__main__":
    main()

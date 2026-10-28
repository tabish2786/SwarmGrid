# Conflict ranker model

`conflict_ranker.txt` is the model behind `ConflictSelection::Learned` (C++),
`conflict_selection="learned"` (Python) and "CBS + ML conflict order" on the control page. The same
weights are compiled into the library from `src/conflict_ranker_model.inc`; both files are written
by `scripts/train_conflict_ranker.py`.

## What it is

At every CBS node, each conflict is described by 18 integer features (type, MDD cardinality, time,
MDD widths, path costs and delays, constraint counts, distances to goal; see `ConflictFeature` in
`include/swarmgrid/conflict_ranking.hpp`). The model is a linear score, `sum(weight * feature)`, with
fixed-point integer weights so every platform picks the same conflict. CBS splits the conflict with
the highest score. Optimality does not depend on the model: any conflict choice gives the optimal
plan; the model only changes how big the search tree gets.

It is trained to imitate an oracle that splits every conflict for real and prefers the one whose
cheaper child costs most (the conflict that raises CBS's lower bound most), then the one whose
children cost most together. Training uses pairwise logistic regression on pairs of conflicts from
the same node (a linear RankNet), in the spirit of Huang, Koenig & Dilkina (AAAI 2021).

## How the shipped model was made

```bash
B=./build/swarmgrid
# Round 1: nodes visited by CBS with the M5 cardinal rule
$B ml-data --out r1-large.csv --generated large --agents 40,50,60 --seeds 1001-1060
$B ml-data --out r1-small.csv --generated small --agents 15,20,25 --seeds 1001-1060
python3 scripts/train_conflict_ranker.py r1-large.csv r1-small.csv --model-out r1-model.txt --inc-out r1-model.inc
# Round 2 (DAgger): nodes visited by CBS using the round-1 model, then train on everything
$B ml-data --out r2-large.csv --generated large --agents 40,50,60 --seeds 1001-1060 --collect learned --model r1-model.txt
$B ml-data --out r2-small.csv --generated small --agents 15,20,25 --seeds 1001-1060 --collect learned --model r1-model.txt
python3 scripts/train_conflict_ranker.py r1-large.csv r1-small.csv r2-large.csv r2-small.csv
```

Data collection is deterministic (no time limits; each run stops after the 50 recorded nodes);
each round took under a minute on one core, and each training run about 10 seconds with numpy. Instances with a seed divisible by
5 are held out from training; the held-out numbers are in the header of `conflict_ranker.txt`. The
benchmark instances (seeds 1-25) are never used for training. Re-running the commands above
reproduced the shipped weights exactly on the machine that trained them (another machine's numpy may
differ in the last bits, which can move a rounded weight by one).

The largest weights, on standardised features: is_cardinal +3.57, path_cost_min -2.84, time +2.07,
is_semi_cardinal +2.06, goal_dist_min +1.40, mdd_width_min -1.05, is_target +1.02.

## Caveats

- Imitating the oracle well (it picks the oracle's best conflict at about 98% of held-out nodes) is
  not the same as small trees: the oracle is greedy, and the learned ranker is better than the M5 rule
  on some instances and worse on others. See the README's results for the instance-by-instance count.
- It was trained on two generated warehouse maps only. On very different maps (open spaces, mazes)
  retrain it; the commands above work for any map the CLI can load with `--generated`, and the Python
  package's `conflict_samples` works for any map.

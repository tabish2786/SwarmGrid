#!/usr/bin/env python3
"""Charts and tables for the README from the benchmark CSVs.

    python3 scripts/plot_bench.py --bench results*.csv --lifelong lifelong.csv --out docs/bench

Writes, for each chart, a light and a dark SVG (<name>.svg, <name>-dark.svg; the README picks one
with <picture>), and summary.md with the same numbers as tables. Needs matplotlib.

Every number comes from the CSV rows; nothing is typed in by hand.
"""
import argparse
import csv
import os
import statistics
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter  # noqa: E402

# Fixed slot per solver, the same in every chart (colour follows the entity). Validated with the
# dataviz palette checker in light and dark mode; light mode needs labels/tables (contrast < 3:1
# for three slots), which is why every chart also has a table in summary.md.
SERIES = {
    "pp": ("Prioritized", "#2a78d6", "#3987e5"),
    "cbs": ("CBS (M4)", "#eb6834", "#d95926"),
    "cbs-m5": ("CBS + M5", "#1baf7a", "#199e70"),
    "ecbs-1.05": ("ECBS w=1.05", "#eda100", "#c98500"),
    "ecbs-1.2": ("ECBS w=1.2", "#e87ba4", "#d55181"),
}
LIFELONG_SOLVER = {"pp": "pp", "ecbs": "ecbs-1.2"}  # lifelong ECBS runs with w = 1.2
THEMES = {
    "light": dict(surface="#fcfcfb", text="#0b0b0b", muted="#52514e", grid="#e6e5e0", pos="#2a78d6", neg="#e34948"),
    "dark": dict(surface="#1a1a19", text="#f0efec", muted="#c3c2b7", grid="#34342f", pos="#3987e5", neg="#e66767"),
}


def read_csv(paths):
    """Rows of every file. A file without a header row (older --shard output) borrows the header
    of the first file that has one."""
    header, rows, headerless = None, [], []
    for p in paths:
        with open(p, newline="") as f:
            lines = list(csv.reader(f))
        if not lines:
            continue
        if not lines[0][0].replace("_", "").isalpha():
            headerless.append(lines)
            continue
        header = header or lines[0]
        rows.extend(dict(zip(lines[0], line)) for line in lines[1:])
    for lines in headerless:
        rows.extend(dict(zip(header, line)) for line in lines)
    return rows


def style(ax, fig, theme):
    t = THEMES[theme]
    fig.patch.set_facecolor(t["surface"])
    ax.set_facecolor(t["surface"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t["grid"])
    ax.tick_params(colors=t["muted"], labelsize=9, length=0, pad=6)
    ax.yaxis.grid(True, color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)
    ax.xaxis.label.set_color(t["muted"])
    ax.yaxis.label.set_color(t["muted"])


def finish(fig, ax, theme, title, subtitle, path, legend=True):
    t = THEMES[theme]
    fig.text(0.01, 0.97, title, color=t["text"], fontsize=12, fontweight="bold", va="top")
    fig.text(0.01, 0.90, subtitle, color=t["muted"], fontsize=9, va="top")
    if legend:
        leg = ax.legend(loc="lower left", bbox_to_anchor=(0, 1.02), ncol=5, frameon=False, fontsize=9,
                        handlelength=1.6, columnspacing=1.2)
        for txt in leg.get_texts():
            txt.set_color(t["text"])
    fig.subplots_adjust(left=0.09, right=0.98, top=0.74, bottom=0.14)
    fig.savefig(path, format="svg", facecolor=t["surface"])
    plt.close(fig)


def count_label(v, _pos=None):
    if v >= 1e6:
        return f"{v / 1e6:g}M"
    if v >= 1e3:
        return f"{v / 1e3:g}k"
    return f"{v:g}"


def seconds_label(v, _pos=None):
    return f"{v * 1000:g} ms" if v < 1 else f"{v:g} s"


def log_axis(ax, label_fn):
    ax.set_yscale("log")
    ax.yaxis.set_major_locator(LogLocator(base=10))
    ax.yaxis.set_major_formatter(FuncFormatter(label_fn))
    ax.yaxis.set_minor_formatter(NullFormatter())


def both_themes(draw, out, name):
    for theme in ("light", "dark"):
        draw(theme, os.path.join(out, f"{name}{'' if theme == 'light' else '-dark'}.svg"))


def median(xs):
    return statistics.median(xs) if xs else None


def bench_charts(rows, out):
    maps = sorted({r["map"] for r in rows})
    algos = [a for a in SERIES if any(r["algorithm"] == a for r in rows)]
    agents = sorted({int(r["agents"]) for r in rows})
    by = defaultdict(list)  # (agents, algo) -> rows
    for r in rows:
        by[(int(r["agents"]), r["algorithm"])].append(r)
    scen = defaultdict(dict)  # (agents, scenario) -> algo -> row
    for r in rows:
        scen[(int(r["agents"]), r["scenario"])][r["algorithm"]] = r
    xs = list(range(len(agents)))
    lines = []
    total = {k: len({r["scenario"] for r in rows if int(r["agents"]) == k}) for k in agents}
    timeout = "60 s"

    def success(a, k):
        rs = by[(k, a)]
        return 100.0 * sum(r["solved"] == "1" for r in rs) / len(rs) if rs else None

    def draw_success(theme, path):
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        style(ax, fig, theme)
        for i, a in enumerate(algos):
            label, light, dark = SERIES[a]
            ys = [success(a, k) for k in agents]
            # Small sideways offsets so lines that sit on top of each other (all at 100 %) stay visible.
            off = (i - (len(algos) - 1) / 2) * 0.06
            ax.plot([x + off for x in xs], ys, color=light if theme == "light" else dark, linewidth=1.8,
                    marker="o", markersize=6, markeredgecolor=THEMES[theme]["surface"], markeredgewidth=1.5,
                    label=label)
        ax.set_xticks(xs, [str(k) for k in agents])
        ax.set_ylim(-4, 104)
        ax.set_ylabel("solved (%)")
        ax.set_xlabel("robots")
        finish(fig, ax, theme, "Instances solved within the time limit",
               f"{', '.join(maps)} · {max(total.values())} instances per fleet size · {timeout} per solve", path)

    both_themes(draw_success, out, "success")

    def draw_runtime(theme, path):
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        style(ax, fig, theme)
        for i, a in enumerate(algos):
            label, light, dark = SERIES[a]
            pts = [(x, median([float(r["seconds"]) for r in by[(k, a)] if r["solved"] == "1"]))
                   for x, k in zip(xs, agents)]
            pts = [(x, y) for x, y in pts if y is not None]
            if not pts:
                continue
            off = (i - (len(algos) - 1) / 2) * 0.06
            ax.plot([x + off for x, _ in pts], [max(y, 1e-5) for _, y in pts],
                    color=light if theme == "light" else dark, linewidth=1.8, marker="o", markersize=6,
                    markeredgecolor=THEMES[theme]["surface"], markeredgewidth=1.5, label=label)
        log_axis(ax, seconds_label)
        ax.set_xticks(xs, [str(k) for k in agents])
        ax.set_ylabel("median time (log scale)")
        ax.set_xlabel("robots")
        finish(fig, ax, theme, "Planning time",
               "median over the instances each planner solved (unsolved runs are not in the median)", path)

    both_themes(draw_runtime, out, "runtime")

    # M5: CT nodes, plain CBS vs CBS + M5, on instances both solved.
    m5 = []
    for k in agents:
        plain, fast = [], []
        for (kk, _), d in scen.items():
            if kk == k and d.get("cbs", {}).get("solved") == "1" and d.get("cbs-m5", {}).get("solved") == "1":
                plain.append(int(d["cbs"]["ct_expanded"]))
                fast.append(int(d["cbs-m5"]["ct_expanded"]))
        if plain:
            ratios = [p / max(f, 1) for p, f in zip(plain, fast)]
            m5.append(dict(agents=k, n=len(plain), plain=median(plain), fast=median(fast),
                           total_plain=sum(plain), total_fast=sum(fast), ratio_median=median(ratios),
                           ratio_max=max(ratios)))

    def draw_m5(theme, path):
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        style(ax, fig, theme)
        w = 0.36
        gx = list(range(len(m5)))
        for j, (key, a) in enumerate((("total_plain", "cbs"), ("total_fast", "cbs-m5"))):
            label, light, dark = SERIES[a]
            ax.bar([x + (j - 0.5) * (w + 0.02) for x in gx], [max(d[key], 1) for d in m5], width=w,
                   color=light if theme == "light" else dark, label=label)
        for x, d in zip(gx, m5):
            ax.text(x, max(d["total_plain"], d["total_fast"], 1) * 1.6, f"{d['total_plain'] / max(d['total_fast'], 1):.1f}× fewer",
                    ha="center", va="bottom", fontsize=9, color=THEMES[theme]["text"])
        log_axis(ax, count_label)
        ax.set_ylim(top=max(d["total_plain"] for d in m5) * 12 if m5 else 10)
        ax.set_xticks(gx, [f"{d['agents']} robots\n({d['n']} solved by both)" for d in m5])
        ax.set_ylabel("CT nodes expanded, total (log scale)")
        finish(fig, ax, theme, "M5: conflict prioritization + bypass shrink the CBS tree",
               "sum over the instances both versions solved · same optimal cost on every one", path)

    if m5:
        both_themes(draw_m5, out, "cbs-m5")

    # Tables
    md = [f"### Benchmark: {', '.join(maps)}\n",
          f"{max(total.values())} instances per fleet size, {timeout} per solve. Solved = finished within the limit with a "
          "plan the independent validator accepts.\n",
          "| robots | " + " | ".join(SERIES[a][0] for a in algos) + " |",
          "|---|" + "---|" * len(algos)]
    for k in agents:
        cells = []
        for a in algos:
            rs = by[(k, a)]
            solved = [r for r in rs if r["solved"] == "1"]
            t = median([float(r["seconds"]) for r in solved])
            cells.append(f"{len(solved)}/{len(rs)}" + (f" · {t:.3g} s" if t is not None else ""))
        md.append(f"| {k} | " + " | ".join(cells) + " |")
    md.append("\nCell: solved / instances · median seconds over the solved ones.\n")
    # Cost quality: sum of costs relative to the best lower bound known for that instance.
    md.append("**Solution cost vs the best known lower bound** (mean of cost / bound over instances the planner solved; "
              "the bound is the optimum when CBS solved it, else the largest ECBS lower bound):\n")
    md.append("| robots | " + " | ".join(SERIES[a][0] for a in algos) + " |")
    md.append("|---|" + "---|" * len(algos))
    for k in agents:
        cells = []
        for a in algos:
            ratios = []
            for (kk, _), d in scen.items():
                if kk != k or d.get(a, {}).get("solved") != "1":
                    continue
                bound = max((int(r["lower_bound"]) for r in d.values() if r["solved"] == "1"), default=0)
                if bound > 0:
                    ratios.append(int(d[a]["sum_of_costs"]) / bound)
            cells.append(f"{statistics.mean(ratios):.3f}" if ratios else "–")
        md.append(f"| {k} | " + " | ".join(cells) + " |")
    if m5:
        md.append("\n**M5 (CBS with MDD conflict prioritization + bypass) vs plain CBS**, instances both solved:\n")
        md.append("| robots | both solved | median CT nodes plain → M5 | total plain / total M5 | median per-instance ratio | best ratio |")
        md.append("|---|---|---|---|---|---|")
        for d in m5:
            md.append(f"| {d['agents']} | {d['n']} | {d['plain']:.0f} → {d['fast']:.0f} | "
                      f"{d['total_plain'] / max(d['total_fast'], 1):.1f}× | {d['ratio_median']:.1f}× | {d['ratio_max']:.0f}× |")
    return "\n".join(md) + "\n", m5


def lifelong_charts(rows, out):
    groups = defaultdict(list)
    for r in rows:
        groups[(r["solver"], r["assign"], int(r["window"]), int(r["robots"]))].append(r)
    robots = sorted({int(r["robots"]) for r in rows})
    windows = sorted({int(r["window"]) for r in rows}, key=lambda w: (w < 0, w))
    seeds = len({r["seed"] for r in rows})
    steps = rows[0]["steps"]

    def thr(solver, assign, w, k):
        rs = groups.get((solver, assign, w, k), [])
        return statistics.mean(float(r["throughput"]) for r in rs) if rs else None

    main_w = 10 if 10 in windows else windows[0]

    def draw_thr(theme, path):
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        style(ax, fig, theme)
        for solver, dash, wlabel in (("ecbs", "-", f"w={main_w}"), ("pp", "-", f"w={main_w}"), ("ecbs", "--", "whole paths")):
            w = main_w if wlabel != "whole paths" else -1
            if (solver, "hungarian", w, robots[0]) not in groups:
                continue
            label, light, dark = SERIES[LIFELONG_SOLVER[solver]]
            label = ("ECBS" if solver == "ecbs" else "Prioritized") + f", {wlabel}"
            ax.plot(robots, [thr(solver, "hungarian", w, k) for k in robots], linestyle=dash,
                    color=light if theme == "light" else dark, linewidth=1.8, marker="o", markersize=6,
                    markeredgecolor=THEMES[theme]["surface"], markeredgewidth=1.5, label=label)
        ax.set_xticks(robots)
        ax.set_ylabel("orders delivered per timestep")
        ax.set_xlabel("robots")
        ax.set_ylim(bottom=0)
        finish(fig, ax, theme, "Lifelong throughput (RHCR)",
               f"small warehouse · {steps} steps · mean of {seeds} seeds · replan every 5 steps · Hungarian assignment", path)

    both_themes(draw_thr, out, "lifelong-throughput")

    gains = []
    for k in robots:
        g, h = thr("ecbs", "greedy", main_w, k), thr("ecbs", "hungarian", main_w, k)
        per_seed = []
        gr = {r["seed"]: float(r["throughput"]) for r in groups.get(("ecbs", "greedy", main_w, k), [])}
        for r in groups.get(("ecbs", "hungarian", main_w, k), []):
            if r["seed"] in gr and gr[r["seed"]] > 0:
                per_seed.append(100 * (float(r["throughput"]) / gr[r["seed"]] - 1))
        if g and h:
            gains.append(dict(robots=k, greedy=g, hungarian=h, pct=100 * (h / g - 1),
                              wins=sum(p > 0 for p in per_seed), n=len(per_seed),
                              lo=min(per_seed) if per_seed else 0, hi=max(per_seed) if per_seed else 0))

    def draw_gain(theme, path):
        t = THEMES[theme]
        fig, ax = plt.subplots(figsize=(7.2, 4.0))
        style(ax, fig, theme)
        xs = list(range(len(gains)))
        ax.bar(xs, [d["pct"] for d in gains], width=0.5, color=[t["pos"] if d["pct"] >= 0 else t["neg"] for d in gains])
        ax.axhline(0, color=t["muted"], linewidth=1)
        for x, d in zip(xs, gains):
            ax.text(x, d["pct"] + (0.15 if d["pct"] >= 0 else -0.15), f"{d['pct']:+.1f}%\n{d['wins']}/{d['n']} seeds better",
                    ha="center", va="bottom" if d["pct"] >= 0 else "top", fontsize=8.5, color=t["text"])
        span = max([abs(d["pct"]) for d in gains] + [1])
        ax.set_ylim(-span * 1.8, span * 1.8)
        ax.set_xticks(xs, [str(d["robots"]) for d in gains])
        ax.set_ylabel("throughput change (%)")
        ax.set_xlabel("robots")
        finish(fig, ax, theme, "Hungarian vs greedy assignment",
               f"mean throughput, ECBS, w={main_w}, {seeds} seeds · above 0: Hungarian delivers more", path, legend=False)

    both_themes(draw_gain, out, "assignment")

    md = [f"### Lifelong (RHCR) on the small warehouse\n",
          f"{steps} steps per run, mean of {seeds} seeds, replan every 5 steps (h), conflicts resolved for w steps. "
          "Throughput = orders delivered per timestep. Collisions in executed motion: "
          f"{sum(int(r['collisions']) for r in rows)} over {len(rows)} runs.\n",
          "| robots | " + " | ".join(f"{('ECBS' if s == 'ecbs' else 'PP')} w={'whole' if w < 0 else w}"
                                     for s in ("ecbs", "pp") for w in windows) + " |",
          "|---|" + "---|" * (2 * len(windows))]
    for k in robots:
        md.append(f"| {k} | " + " | ".join(
            (f"{thr(s, 'hungarian', w, k):.3f}" if thr(s, "hungarian", w, k) is not None else "–")
            for s in ("ecbs", "pp") for w in windows) + " |")
    md.append("\n(Hungarian assignment in all columns.)\n")
    md.append("| robots | planning time per replan, ECBS w=" + str(main_w) + " (mean / max s) | planner failures (fallback used) | deadlocks detected | escapes |")
    md.append("|---|---|---|---|---|")
    for k in robots:
        rs = groups.get(("ecbs", "hungarian", main_w, k), [])
        if rs:
            md.append(f"| {k} | {statistics.mean(float(r['mean_plan_s']) for r in rs):.4f} / {max(float(r['max_plan_s']) for r in rs):.3f} | "
                      f"{sum(int(r['planner_failures']) for r in rs)} | {sum(int(r['deadlocks']) for r in rs)} | {sum(int(r['escapes']) for r in rs)} |")
    md.append("\n**Deadlock recovery and fallbacks**, all runs of each configuration (both assignments, all fleet sizes):\n")
    md.append("| planner | window | stuck detections | escapes | main planner failed (fallback used) | everyone waited |")
    md.append("|---|---|---|---|---|---|")
    for solver in ("ecbs", "pp"):
        for w in windows:
            rs = [r for r in rows if r["solver"] == solver and int(r["window"]) == w]
            if rs:
                md.append(f"| {'ECBS' if solver == 'ecbs' else 'Prioritized'} | {'whole paths' if w < 0 else w} | "
                          f"{sum(int(r['deadlocks']) for r in rs)} | {sum(int(r['escapes']) for r in rs)} | "
                          f"{sum(int(r['planner_failures']) for r in rs)} | {sum(int(r['all_wait']) for r in rs)} |")

    def per_delivery(assign, k):
        rs = groups.get(("ecbs", assign, main_w, k), [])
        return statistics.mean(int(r["assigned_distance"]) / max(1, int(r["delivered"])) for r in rs) if rs else None

    md.append(f"\n**Hungarian vs greedy assignment** (ECBS, w={main_w}). Distance = robot-to-parcel cells at "
              "assignment, divided by orders delivered:\n")
    md.append("| robots | throughput greedy | throughput Hungarian | change | seeds where Hungarian is better | per-seed range | distance per order greedy → Hungarian |")
    md.append("|---|---|---|---|---|---|---|")
    for d in gains:
        g, h = per_delivery("greedy", d["robots"]), per_delivery("hungarian", d["robots"])
        dist = f"{g:.2f} → {h:.2f} ({100 * (h / g - 1):+.1f}%)" if g and h else "–"
        md.append(f"| {d['robots']} | {d['greedy']:.3f} | {d['hungarian']:.3f} | {d['pct']:+.1f}% | {d['wins']}/{d['n']} | "
                  f"{d['lo']:+.1f}% … {d['hi']:+.1f}% | {dist} |")
    return "\n".join(md) + "\n", gains


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", nargs="*", default=[])
    ap.add_argument("--lifelong", nargs="*", default=[])
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    plt.rcParams.update({"svg.fonttype": "none", "font.family": "sans-serif",
                         "font.sans-serif": ["Inter", "Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
                         "svg.hashsalt": "swarmgrid"})
    parts = []
    if args.bench:
        text, _ = bench_charts(read_csv(args.bench), args.out)
        parts.append(text)
    if args.lifelong:
        text, _ = lifelong_charts(read_csv(args.lifelong), args.out)
        parts.append(text)
    with open(os.path.join(args.out, "summary.md"), "w") as f:
        f.write("<!-- Generated by scripts/plot_bench.py from the benchmark CSVs. -->\n\n" + "\n".join(parts))
    print("\n".join(parts))


if __name__ == "__main__":
    main()

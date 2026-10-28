"""Compare the planners on growing fleets.   python compare_planners.py"""
import swarmgrid as sg

grid = sg.warehouse(8, 8, 10, 2, 2, 3)                  # the 100 x 36 benchmark warehouse
methods = [("cbs", {}), ("cbs", {"conflict_selection": "learned"}), ("ecbs", {"weight": 1.2}), ("prioritized", {})]
print(f"{'robots':>6}  {'planner':<16} {'solved':>6} {'sum of costs':>12} {'time (ms)':>9}")
for robots in (10, 30, 50):
    starts, goals = sg.random_agents(grid, robots, seed=1)
    for method, extra in methods:
        plan = sg.solve(grid, starts, goals, method, time_limit=10, **extra)
        name = method + ("-learned" if extra.get("conflict_selection") == "learned" else "")
        cost = plan.sum_of_costs if plan.solved else "-"
        print(f"{robots:>6}  {name:<16} {str(plan.solved):>6} {cost:>12} {plan.seconds * 1000:>9.1f}")

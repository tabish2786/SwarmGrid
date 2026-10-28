"""Plan 10 warehouse robots optimally and check the plan.   python quickstart.py"""
import swarmgrid as sg

grid = sg.warehouse()                                   # 27 x 21 generated warehouse
starts, goals = sg.random_agents(grid, 10, seed=42)     # same robots as the web demo

plan = sg.solve(grid, starts, goals)                    # optimal CBS
print(f"CBS: sum of costs {plan.sum_of_costs}, makespan {plan.makespan}, "
      f"{plan.nodes_expanded} tree nodes, {plan.seconds * 1000:.1f} ms")
print("valid:", sg.validate(grid, starts, goals, plan.paths).ok)

naive = sg.solve(grid, starts, goals, method="independent")
report = sg.validate(grid, starts, goals, naive.paths)
print(f"without coordination: {len(report.conflicts)} collisions, e.g. {report.conflicts[0]}")

print(sg.render(grid, starts))

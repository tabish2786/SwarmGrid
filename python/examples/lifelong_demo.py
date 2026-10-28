"""A warehouse running for 500 steps with a stream of orders.   python lifelong_demo.py"""
import swarmgrid as sg

grid = sg.warehouse()
for assignment in ("greedy", "hungarian"):
    stats = sg.simulate_lifelong(grid, robots=30, steps=500, window=10, replan_every=5,
                                 solver="ecbs", assignment=assignment, breakdown_rate=0.001)
    print(f"{assignment:>9}: {stats.delivered} orders, {stats.throughput:.3f} per step, "
          f"{stats.breakdowns} breakdowns, {stats.collisions} collisions, "
          f"{stats.mean_plan_seconds * 1000:.1f} ms per replan")

"""Tests for the swarmgrid Python package. Run after `pip install .`:

    python -m unittest discover -s tests/python -v

The expected numbers are the same golden values the C++ tests pin (tests/test_cbs.cpp etc.), so
these tests also check that Python gets exactly what C++ computes.
"""
import os
import tempfile
import threading
import unittest

import swarmgrid as sg


class MapsAndAgents(unittest.TestCase):
    def test_warehouse_defaults(self):
        g = sg.warehouse()
        self.assertEqual((g.width, g.height, g.count_obstacles()), (27, 21, 240))
        self.assertEqual(len(g.rows()), 21)
        self.assertEqual(len(g.free_cells()), 27 * 21 - 240)
        big = sg.warehouse(8, 8, 10, 2, 2, 3)
        self.assertEqual((big.width, big.height, big.count_obstacles()), (100, 36, 1280))

    def test_grid_from_rows_and_editing(self):
        g = sg.grid_from_rows(["....", ".@@.", "...."])
        self.assertEqual((g.width, g.height, g.count_obstacles()), (4, 3, 2))
        self.assertFalse(g.is_free(1, 1))
        self.assertFalse(g.is_free(9, 9))  # outside counts as not free
        h = g.copy()
        h.set_obstacle(0, 0)
        self.assertTrue(g.is_free(0, 0))
        self.assertFalse(h.is_free(0, 0))
        self.assertNotEqual(g, h)
        with self.assertRaises(ValueError):
            sg.grid_from_rows(["..", "..."])  # ragged
        with self.assertRaises(ValueError):
            h.set_obstacle(5, 5)

    def test_random_agents_are_deterministic(self):
        g = sg.warehouse()
        self.assertEqual(sg.random_agents(g, 10, seed=42), sg.random_agents(g, 10, seed=42))
        self.assertNotEqual(sg.random_agents(g, 10, seed=42), sg.random_agents(g, 10, seed=43))
        starts, goals = sg.random_agents(g, 10, seed=42)
        self.assertEqual(len(set(starts)), 10)
        self.assertEqual(len(set(goals)), 10)
        self.assertTrue(all(g.is_free(x, y) for x, y in starts + goals))

    def test_movingai_files(self):
        with tempfile.TemporaryDirectory() as d:
            m = os.path.join(d, "t.map")
            with open(m, "w") as f:
                f.write("type octile\nheight 2\nwidth 3\nmap\n...\n.@.\n")
            s = os.path.join(d, "t.scen")
            with open(s, "w") as f:
                f.write("version 1\n0\tt.map\t3\t2\t0\t0\t2\t1\t3\n0\tt.map\t3\t2\t2\t0\t0\t1\t3\n")
            g = sg.load_map(m)
            self.assertEqual(g.rows(), ["...", ".@."])
            self.assertEqual(sg.load_scenario(g, s, 2), ([(0, 0), (2, 0)], [(2, 1), (0, 1)]))
            with self.assertRaises(FileNotFoundError):
                sg.load_map(os.path.join(d, "missing.map"))

    def test_shortest_path_and_render(self):
        g = sg.grid_from_rows(["...", "@@.", "..."])
        self.assertEqual(sg.shortest_path(g, (0, 0), (0, 2)), [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)])
        self.assertIsNone(sg.shortest_path(sg.grid_from_rows([".@."]), (0, 0), (2, 0)))
        self.assertIn("0", sg.render(g, [(0, 0)]))
        with self.assertRaises(ValueError):
            sg.shortest_path(g, (0, 0), (0, 1))  # goal on a shelf


class Planning(unittest.TestCase):
    def setUp(self):
        self.g = sg.warehouse()
        self.starts, self.goals = sg.random_agents(self.g, 10, seed=42)

    def test_cbs_golden_numbers(self):
        plan = sg.solve(self.g, self.starts, self.goals)
        self.assertTrue(plan)
        self.assertEqual((plan.sum_of_costs, plan.makespan, plan.nodes_expanded), (208, 38, 245))
        self.assertEqual(plan.lower_bound, 208)
        self.assertTrue(sg.validate(self.g, self.starts, self.goals, plan.paths))
        plain = sg.solve(self.g, self.starts, self.goals, conflict_selection="first", bypass=False)
        self.assertEqual((plain.sum_of_costs, plain.nodes_expanded), (208, 494))

    def test_all_conflict_selections_are_optimal(self):
        for selection in ("first", "cardinal", "learned", "oracle"):
            with self.subTest(selection=selection):
                plan = sg.solve(self.g, self.starts, self.goals, conflict_selection=selection)
                self.assertEqual(plan.sum_of_costs, 208)

    def test_custom_ranker(self):
        names = sg.conflict_feature_names()
        self.assertEqual(len(names), 18)
        self.assertEqual(len(sg.builtin_ranker_weights()), 18)
        zeros = sg.solve(self.g, self.starts, self.goals, conflict_selection="learned", model=[0] * 18)
        first = sg.solve(self.g, self.starts, self.goals, conflict_selection="first")
        self.assertEqual(zeros.nodes_expanded, first.nodes_expanded)  # all ties: earliest conflict
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "m.txt")
            with open(path, "w") as f:
                f.write("".join(f"{n} {i}\n" for i, n in enumerate(names)))
            self.assertEqual(sg.load_ranker(path), list(range(18)))
            self.assertTrue(sg.solve(self.g, self.starts, self.goals, conflict_selection="learned", model=path))
        with self.assertRaises(ValueError):
            sg.solve(self.g, self.starts, self.goals, conflict_selection="learned", model=[1, 2])

    def test_other_methods(self):
        pri = sg.solve(self.g, self.starts, self.goals, "prioritized")
        self.assertEqual(pri.sum_of_costs, 212)
        self.assertGreater(pri.seconds, 0.0)  # measured, not a placeholder
        self.assertTrue(sg.validate(self.g, self.starts, self.goals, pri.paths))
        ind = sg.solve(self.g, self.starts, self.goals, "independent")
        self.assertEqual(ind.sum_of_costs, 200)
        report = sg.validate(self.g, self.starts, self.goals, ind.paths)
        self.assertFalse(report)
        self.assertEqual(len(report.conflicts), 16)
        self.assertEqual({c.type for c in report.conflicts}, {"vertex", "edge", "target"})
        for w in (1.0, 1.2):
            ecbs = sg.solve(self.g, self.starts, self.goals, "ecbs", weight=w)
            self.assertTrue(ecbs)
            self.assertLessEqual(ecbs.sum_of_costs, w * ecbs.lower_bound + 1e-9)
            self.assertGreaterEqual(ecbs.sum_of_costs, 208)
        self.assertEqual(sg.solve(self.g, self.starts, self.goals, "ecbs", weight=1.0).sum_of_costs, 208)

    def test_limits_and_errors(self):
        tight = sg.solve(self.g, self.starts, self.goals, conflict_selection="first", bypass=False, max_nodes=10)
        self.assertFalse(tight)
        self.assertTrue(tight.limit_reached)
        with self.assertRaises(ValueError):
            sg.solve(self.g, self.starts, self.goals, "astar")
        with self.assertRaises(ValueError):
            sg.solve(self.g, self.starts, self.goals, conflict_selection="best")
        with self.assertRaises(ValueError):
            sg.solve(self.g, [(0, 0), (0, 0)], [(1, 0), (2, 0)])  # shared start
        with self.assertRaises(ValueError):
            sg.solve(self.g, [(99, 0)], [(1, 0)])  # outside

    def test_validate_catches_a_teleport(self):
        g = sg.grid_from_rows(["....."])
        report = sg.validate(g, [(0, 0)], [(4, 0)], [[(0, 0), (4, 0)]])
        self.assertFalse(report.ok)
        self.assertTrue(any("jumps" in e for e in report.errors))

    def test_parallel_threads(self):
        results = []

        def work():
            results.append(sg.solve(self.g, self.starts, self.goals).sum_of_costs)

        threads = [threading.Thread(target=work) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(results, [208] * 4)


class OrdersAndLifelong(unittest.TestCase):
    def test_hungarian_beats_greedy(self):
        g = sg.grid_from_rows(["......"])
        greedy = sg.assign(g, [(0, 0), (3, 0)], [(2, 0), (5, 0)], "greedy")
        best = sg.assign(g, [(0, 0), (3, 0)], [(2, 0), (5, 0)])
        self.assertEqual(sum(a.distance for a in greedy), 6)
        self.assertEqual(sum(a.distance for a in best), 4)
        self.assertEqual(best[0], sg.Assignment(robot=0, order=0, distance=2))

    def test_lifelong_is_safe_and_deterministic(self):
        g = sg.warehouse()
        a = sg.simulate_lifelong(g, robots=10, steps=150, window=8, replan_every=4, record_positions=True)
        b = sg.simulate_lifelong(g, robots=10, steps=150, window=8, replan_every=4, record_positions=True)
        self.assertEqual(a.collisions, 0)
        self.assertGreater(a.delivered, 5)
        self.assertEqual(len(a.positions), 151)
        self.assertEqual(a.positions, b.positions)
        for t in range(1, len(a.positions)):
            self.assertEqual(len(set(a.positions[t])), 10)  # nobody shares a cell
        broken = sg.simulate_lifelong(g, robots=10, steps=150, breakdown_rate=0.01, solver="prioritized",
                                      assignment="greedy", window=None)
        self.assertEqual(broken.collisions, 0)
        self.assertGreater(broken.breakdowns, 0)


class ConflictLearningData(unittest.TestCase):
    def test_samples_match_the_mdd_classification(self):
        g = sg.warehouse()
        starts, goals = sg.random_agents(g, 15, seed=7)
        data = sg.conflict_samples(g, starts, goals, nodes=20, conflicts=8)
        self.assertGreater(len(data), 10)
        card = data.feature_names.index("is_cardinal")
        semi = data.feature_names.index("is_semi_cardinal")
        for f, lo, total in zip(data.features, data.min_increase, data.sum_increase):
            if f[card]:
                self.assertGreaterEqual(lo, 1)
            elif f[semi]:
                self.assertEqual(lo, 0)
                self.assertGreaterEqual(total, 1)
            else:
                self.assertEqual(total, 0)


if __name__ == "__main__":
    unittest.main()

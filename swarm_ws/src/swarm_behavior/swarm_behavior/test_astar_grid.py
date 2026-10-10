"""Pruebas puras de geometría y A*; no requieren ROS 2."""
import math
import unittest

from astar_grid import (COLUMN_CLEARANCE, CLEARANCE, RESOLUTION,
                        WarehouseGrid)


class TestAstarGrid(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = WarehouseGrid()

    def test_dimensions(self):
        self.assertEqual((self.grid.nx, self.grid.ny), (112, 192))
        self.assertEqual(len(self.grid.free), 192)
        self.assertEqual(len(self.grid.free[0]), 112)

    def test_boundaries_are_inflated(self):
        self.assertFalse(self.grid.is_free_point(-14, 0))
        self.assertFalse(self.grid.is_free_point(0, 23.5))
        self.assertTrue(self.grid.is_free_point(-14 + CLEARANCE + 0.05, 0))

    def test_columns_are_inflated(self):
        self.assertFalse(self.grid.is_free_point(7.4, -7.5))
        self.assertFalse(self.grid.is_free_point(7.4 + COLUMN_CLEARANCE - 0.03, -7.5))
        self.assertTrue(self.grid.is_free_point(7.4 + COLUMN_CLEARANCE + 0.03, -7.5))

    def test_segment_between_free_endpoints_may_be_blocked(self):
        a = (5.8, -7.5)
        b = (9.0, -7.5)
        self.assertTrue(self.grid.is_free_point(*a))
        self.assertTrue(self.grid.is_free_point(*b))
        self.assertFalse(self.grid.is_clear_segment(a, b))

    def test_plan_across_column(self):
        start = (1.401336018, -3.934677470)  # spawn teórico, seed 42, 1 robot
        goal = (9.5, -8.5)
        result = self.grid.astar(start, goal)
        self.assertGreater(len(result.cells), 2)
        self.assertGreater(result.cost_m, math.dist(start, goal) - RESOLUTION * 2)
        # Cada vecino de A* es legal; no atraviesa la huella de la columna.
        for a, b in zip(result.cells, result.cells[1:]):
            self.assertLessEqual(max(abs(a[0]-b[0]), abs(a[1]-b[1])), 1)
            self.assertTrue(self.grid.is_clear_segment(
                self.grid.to_world(a), self.grid.to_world(b)))
        waypoints = self.grid.simplify(start, goal, result.cells)
        self.assertLessEqual(len(waypoints), len(result.cells))
        self.assertEqual(waypoints[-1], goal)
        self.assertFalse(self.grid.is_clear_segment(start, goal),
                         'La meta demo debe obligar a bordear la columna')
        for a, b in zip([start]+waypoints[:-1], waypoints):
            self.assertTrue(self.grid.is_clear_segment(a, b))

    def test_same_cell_start_and_goal(self):
        a = (0.01, 0.01)
        b = (0.02, 0.02)
        result = self.grid.astar(a, b)
        self.assertEqual(len(result.cells), 1)
        self.assertEqual(self.grid.simplify(a, b, result.cells), [b])

    def test_rejects_start_in_column(self):
        with self.assertRaises(ValueError):
            self.grid.astar((7.4, 0), (0, 0))

    def test_rejects_goal_outside_world(self):
        with self.assertRaises(ValueError):
            self.grid.astar((0, 0), (14.5, 0))

    def test_admissible_heuristic_is_eight_connected(self):
        # En un espacio despejado: 4 diagonales y 2 cardinales = costo exacto.
        a = (20, 30)
        b = (24, 36)
        cost = self.grid.heuristic(a, b)
        self.assertAlmostEqual(cost, RESOLUTION * (4*math.sqrt(2) + 2))


if __name__ == '__main__':
    unittest.main(verbosity=2)

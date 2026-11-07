"""Regresión: contactos tangentes sí; penetraciones y separadores inválidos no.

No se usa Gazebo; las restricciones de seguridad permanecen iguales.
"""
from __future__ import annotations
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts' / 'supermarket_seed_pipeline'))
from refinement_supermarket import halfspaces_from_curves


class CircularTangencyTest(unittest.TestCase):
    def _generate(self, x: float):
        controls = [np.tile(np.array([x, 0.0]), (1, 6, 1))]
        # One stationary robot, one piece, one circular exclusion.
        return halfspaces_from_curves(
            controls,
            rectangles=[], bounds=(-10.0, 10.0, -10.0, 10.0),
            static_clearance_m=0.6, circles_by_robot=[[((0.0, 0.0), 1.0)]],
            wall_clearance_m=0.6, robot_radius_m=0.575, samples=32)

    def test_exact_tangency_has_valid_halfspace(self):
        hs = self._generate(1.0)
        self.assertEqual(len(hs), 1)
        self.assertEqual(len(hs[0]), 1)
        self.assertEqual(len(hs[0][0]), 5)  # four walls + circular exclusion
        n, bound = hs[0][0][-1]
        self.assertLessEqual(float(np.dot(n, [1.0, 0.0]) - bound), 1e-12)

    def test_genuine_penetration_rejected(self):
        with self.assertRaisesRegex(ValueError, 'sin separador convexo'):
            self._generate(0.9999)

    def test_separated_circle_accepted(self):
        hs = self._generate(1.1)
        self.assertEqual(len(hs[0][0]), 5)


if __name__ == '__main__':
    unittest.main()

"""Regresiones del filtro de accesibilidad de basuras y del presupuesto CBS."""
import importlib.util
import json
from pathlib import Path
import unittest

WS = Path(__file__).resolve().parents[1]
GEN = WS / 'scripts/supermarket_escenarios/generar_escenario.py'
spec = importlib.util.spec_from_file_location('generar_escenario_accessibility', GEN)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class GenerationAccessibilityTest(unittest.TestCase):
    def test_rubber_factory_seeds_have_accessible_trash(self):
        graph = WS / 'config/maps/rubber_factory/grafo.json'
        obstacles = WS / 'config/maps/rubber_factory/obstaculos.json'
        for seed in (1, 4, 6, 2026):
            with self.subTest(seed=seed):
                _, _, scenario = mod.choose(seed, graph_file=graph,
                    geometry_file=obstacles, world='rubber_factory')
                self.assertTrue(mod.check_cbs_accessibility(scenario, graph))
                self.assertEqual(len(scenario['trash']), 6)
                self.assertTrue(scenario['validation']['cbs_dynamic_initial_accessibility'])

    def test_rejects_approach_blocked_by_pending_trash(self):
        graph = WS / 'config/maps/rubber_factory/grafo.json'
        scenario = json.loads((WS/'config/scenarios/rubber_factory/seed_2026.json').read_text())
        # Una basura exactamente sobre un inicio bloquea el nodo del robot
        # en el mapa dinámico; el escenario no debe aceptarse.
        scenario['trash'][0]['xy_m'] = list(scenario['robots'][0]['xy_m'])
        self.assertFalse(mod.check_cbs_accessibility(scenario, graph))

    def test_default_ct_budget_consistent(self):
        full = (WS/'scripts/supermarket_seed_pipeline/run_full_offline.py').read_text()
        cbs = (WS/'scripts/supermarket_seed_pipeline/plan_milp_multigoal.py').read_text()
        self.assertIn("ap.add_argument('--max-ct',type=int,default=None", full)
        self.assertIn('else 10000', full)
        self.assertIn("p.add_argument('--max-ct',type=int,default=10000)", cbs)
        self.assertIn('def plan(assignment, scenario, *, max_ct=10000', cbs)


if __name__ == '__main__':
    unittest.main()

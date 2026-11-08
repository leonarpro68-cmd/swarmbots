"""Regresiones: basura continua debe tener una aproximación de 0.55–2.70 m.

No ejecuta ROS ni modifica escenarios o planes ya guardados.
"""
import importlib.util
import math
from pathlib import Path
import unittest

WS = Path(__file__).resolve().parents[1]
GEN = WS / 'scripts/supermarket_escenarios/generar_escenario.py'
SPEC = importlib.util.spec_from_file_location('generar_escenario_test_distance', GEN)
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


class TrashApproachDistanceTest(unittest.TestCase):
    def test_original_seed1_A2_is_rejected_at_generation(self):
        graph = WS / 'config/maps/rubber_factory/grafo.json'
        obstacles = WS / 'config/maps/rubber_factory/obstaculos.json'
        _, geometry, _, _, component = generator.load(graph, obstacles)
        old_a2 = (-6.64881, -18.61339)
        minimum = min(math.dist(old_a2, q) for q in component)
        self.assertGreater(minimum, generator.PICKUP_D8_MAX_M)
        self.assertAlmostEqual(minimum, 5.400538, places=5)
        self.assertIsNone(generator.nearby_pickup_node(
            old_a2, sorted(component), geometry['obstacles'], clearance=0.75))

    def test_rubber_factory_seeds_have_explicit_near_approach(self):
        graph = WS / 'config/maps/rubber_factory/grafo.json'
        obstacles = WS / 'config/maps/rubber_factory/obstaculos.json'
        for seed in (1, 4, 6, 2026):
            with self.subTest(seed=seed):
                _, geometry, scenario = generator.choose(
                    seed, graph_file=graph, geometry_file=obstacles,
                    world='rubber_factory')
                self.assertTrue(generator.check_cbs_accessibility(scenario, graph))
                self.assertEqual(len(scenario['trash']), 6)
                for trash in scenario['trash']:
                    d = math.dist(trash['xy_m'], trash['approach_node_xy_m'])
                    self.assertGreater(d, generator.PICKUP_D8_MIN_M)
                    self.assertLess(d, generator.PICKUP_D8_MAX_M)
                    self.assertTrue(all(generator.close_to_rect_segment(
                        trash['xy_m'], trash['approach_node_xy_m'], ob, 0.75)
                        for ob in geometry['obstacles']))

    def test_other_maps_still_generate(self):
        for world in ('supermarket', 'colored_warehouse', 'tugbot_warehouse',
                      'pallet_warehouse', 'parking_lot', 'electrical_substation'):
            with self.subTest(world=world):
                seed = 2030 if world == 'electrical_substation' else 2026
                root = WS / 'config/maps' / world
                _, _, scenario = generator.choose(
                    seed, graph_file=root/'grafo.json',
                    geometry_file=root/'obstaculos.json', world=world)
                self.assertEqual(len(scenario['trash']), 6)
                self.assertTrue(all(generator.PICKUP_D8_MIN_M < math.dist(
                    trash['xy_m'], trash['approach_node_xy_m']) < generator.PICKUP_D8_MAX_M
                    for trash in scenario['trash']))

    def test_cbs_ct_and_physical_D8_defaults_are_preserved(self):
        base = WS / 'scripts/supermarket_seed_pipeline'
        full = (base/'run_full_offline.py').read_text()
        cbs = (base/'plan_milp_multigoal.py').read_text()
        preflight = (base/'prepare_phase5.py').read_text()
        executor = (base/'run_auto_experiment_seed.py').read_text()
        visual_grasp = (WS/'scripts/prueba_alinear_agarrar_cbs.py').read_text()
        self.assertIn('else 10000', full)
        self.assertIn("p.add_argument('--max-ct',type=int,default=10000)", cbs)
        self.assertIn('PICKUP_D8_MAX=2.70', preflight)
        self.assertIn("'--max-distance','2.7'", executor)
        self.assertIn('default=2.7,', visual_grasp)

if __name__ == '__main__':
    unittest.main()

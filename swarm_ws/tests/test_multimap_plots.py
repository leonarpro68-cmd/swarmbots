"""Regressions for scenario/map detection and theoretical route backgrounds."""
import importlib.util
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
PLOT_DIR = SCRIPTS / 'supermarket_auto_experiment'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(PLOT_DIR))
from plot_experiment import find_scenario


class MultimapPlotTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ws = Path(self.tmp.name) / 'swarm_ws'
        self.scripts = self.ws / 'scripts'
        (self.scripts / 'supermarket_seed_pipeline' / 'plans').mkdir(parents=True)

    def tearDown(self):
        self.tmp.cleanup()

    def make_case(self, name, seed=456, mode='milp', world_override=None):
        from swarm_maps import scenario_file
        folder = self.ws / 'config' / 'maps' / name
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'grafo.json').write_text('{}')
        (folder / 'obstaculos.json').write_text(json.dumps({
            'bounds': {'xmin': -10, 'xmax': 10, 'ymin': -7, 'ymax': 7},
            'obstacles': [{'name': 'one', 'xmin': 3, 'xmax': 5, 'ymin': 1, 'ymax': 3}]}))
        scenario = scenario_file(self.ws, name, seed)
        scenario.parent.mkdir(parents=True, exist_ok=True)
        scenario.write_text(json.dumps({'seed': seed, 'world': world_override or name}))
        root = self.scripts / 'supermarket_seed_pipeline' / 'plans' / name
        prepared = root / f'seed_{seed}'
        if mode == 'fixed':
            prepared = prepared / 'fixed'
        prepared = prepared / 'prepared'
        prepared.mkdir(parents=True)
        return prepared, scenario

    def test_all_seven_maps_select_their_own_scenarios(self):
        maps = ('supermarket', 'tugbot_warehouse', 'colored_warehouse',
                'pallet_warehouse', 'parking_lot', 'rubber_factory',
                'electrical_substation')
        for name in maps:
            with self.subTest(map=name):
                prepared, expected = self.make_case(name)
                self.assertEqual(find_scenario(prepared, self.scripts), expected)

    def test_fixed_mode_keeps_correct_map(self):
        for name in ('supermarket', 'colored_warehouse'):
            with self.subTest(map=name):
                prepared, expected = self.make_case(name, seed=777, mode='fixed')
                self.assertEqual(find_scenario(prepared, self.scripts), expected)

    def test_mismatched_world_is_rejected_not_silently_supermarket(self):
        prepared, _ = self.make_case('colored_warehouse', world_override='supermarket')
        with self.assertRaisesRegex(ValueError, 'no corresponde'):
            find_scenario(prepared, self.scripts)

    def test_missing_scenario_is_rejected_not_fallback(self):
        prepared, expected = self.make_case('rubber_factory')
        expected.unlink()
        with self.assertRaises(FileNotFoundError):
            find_scenario(prepared, self.scripts)

    def test_render_uses_selected_obstacles_and_bounds(self):
        import numpy as np
        from matplotlib import pyplot as plt
        from plot_routes_with_obstacles import draw_obstacles
        fig, ax = plt.subplots()
        geometry = {'bounds': {'xmin': -14.85, 'xmax': 14.85,
                               'ymin': -11.85, 'ymax': 11.85},
                    'obstacles': [{'name': 'Rojo', 'xmin': -11, 'xmax': -9,
                                   'ymin': 7.5, 'ymax': 8.5}]}
        draw_obstacles(ax, geometry)
        self.assertEqual(len(ax.patches), 1)
        self.assertEqual(tuple(round(x, 2) for x in ax.get_xlim()), (-14.85, 14.85))
        self.assertEqual(tuple(round(y, 2) for y in ax.get_ylim()), (-11.85, 11.85))
        self.assertAlmostEqual(ax.patches[0].get_width(), 2.)
        plt.close(fig)

    def test_routes_only_reuses_existing_windows_without_odom(self):
        import numpy as np
        prepared, _ = self.make_case('electrical_substation', seed=2030)
        # The command is a standalone postprocessor; it should not need
        # events.csv, odom.csv, Gazebo, or even a preflight JSON.
        shutil.copy2(SCRIPTS / 'swarm_maps.py', self.scripts / 'swarm_maps.py')
        run = self.ws / 'scripts' / 'resultados' / 'experiments' / 'electrical_substation' / 'demo'
        run.mkdir(parents=True)
        pts = np.array([np.column_stack((np.linspace(i, i+2, 6),
                                         np.linspace(-2, 1, 6))) for i in range(3)])
        z = run / 'demo.npz'
        np.savez(z, **{f'R{i}_controls': pts[i:i+1] for i in range(3)})
        (run / 'stage_windows_v3.json').write_text(json.dumps({'stages': [
            {'stage': s, 'trajectory': str(z), 'max_duration_s': 3.5}
            for s in ('pickup1', 'delivery1', 'pickup2', 'delivery2')]}))
        proc = subprocess.run([sys.executable, str(PLOT_DIR / 'plot_experiment.py'),
                               '--prepared', str(prepared), '--run-dir', str(run),
                               '--scripts-dir', str(self.scripts), '--only-routes'],
                              check=True, capture_output=True, text=True)
        self.assertIn('ROUTES_PLOT_GENERATED', proc.stdout)
        self.assertIn('electrical_substation', proc.stdout)
        self.assertGreater((run/'plots'/'rutas_teoricas.png').stat().st_size, 10000)
        self.assertFalse((run/'plots'/'velocidad_aceleracion_teoria_vs_odometria.png').exists())

    def test_end_to_end_route_png_contains_world_label(self):
        import numpy as np
        name = 'colored_warehouse'
        _, scenario = self.make_case(name)
        folder = self.ws / 'figures'
        folder.mkdir()
        npz = folder / 'curves.npz'
        controls = {}
        for i in range(3):
            x0 = -6 + 3*i
            pts = np.column_stack((np.linspace(x0, x0 + 2, 6),
                                   np.linspace(-2, 2, 6)))
            controls[f'R{i}_controls'] = pts[None, :, :]
        np.savez(npz, **controls)
        stages = ['pickup1', 'delivery1', 'pickup2', 'delivery2']
        windows = folder / 'stage_windows_v3.json'
        windows.write_text(json.dumps({'stages': [
            {'stage': s, 'trajectory': str(npz), 'max_duration_s': 8.5}
            for s in stages]}))
        out = folder / 'rutas_teoricas.png'
        subprocess.run([sys.executable, str(PLOT_DIR / 'plot_routes_with_obstacles.py'),
                        '--stage-windows', str(windows), '--scenario', str(scenario),
                        '--geometry', str(self.ws/'config/maps'/name/'obstaculos.json'),
                        '--output', str(out), '--mission-label', 'test_seed456'],
                       check=True, capture_output=True, text=True)
        self.assertGreater(out.stat().st_size, 10000)


if __name__ == '__main__':
    unittest.main()

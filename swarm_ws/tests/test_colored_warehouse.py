"""Pruebas de contrato para colored_warehouse; no requieren ROS 2/Gazebo."""
import json
import sys
import unittest
from collections import deque
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'scripts'))
sys.path.insert(0, str(WS / 'scripts' / 'supermarket_escenarios'))
sys.path.insert(0, str(WS / 'src' / 'swarm_worlds' / 'launch'))
from swarm_maps import MAPS, map_files, validate_map, scenario_file, plans_dir
from generar_escenario import choose
from scenario_sync import load_scenario

class ColoredWarehouseTest(unittest.TestCase):
    def test_map_contract_and_topology(self):
        self.assertIn('colored_warehouse', MAPS)
        folder = WS / 'config' / 'maps' / 'colored_warehouse'
        self.assertEqual({p.name for p in folder.iterdir()}, {'grafo.json', 'obstaculos.json'})
        graph, geometry = validate_map(*map_files(WS, 'colored_warehouse'))
        self.assertEqual((len(graph['nodes']), len(graph['edges']), len(geometry['obstacles'])), (301, 874, 14))
        self.assertEqual(graph['metadata']['status'], 'FINAL_TOPOLOGY_APPROVED')
        self.assertEqual(tuple(graph['metadata']['origin']), (-15.0, -12.5))
        self.assertEqual(graph['metadata']['resolution_m'], 1.25)
        self.assertEqual(len(graph['metadata']['local_clearance_exceptions']['edges_ij']), 6)

    def test_central_approved_nodes_and_edges(self):
        graph, _ = validate_map(*map_files(WS, 'colored_warehouse'))
        ids = {(n['i'], n['j']): (n['x'], n['y']) for n in graph['nodes']}
        self.assertIn((11, 10), ids)
        self.assertIn((13, 10), ids)
        edges = {frozenset((tuple(a), tuple(b))) for a, b in graph['edges']}
        self.assertEqual(len(edges), len(graph['edges']))
        exceptions = graph['metadata']['local_clearance_exceptions']['edges_ij']
        for ij1, ij2 in exceptions:
            self.assertIn(frozenset((ids[tuple(ij1)], ids[tuple(ij2)])), edges)
        # La topología seleccionada constituye una única componente conexa.
        nodes = set(ids.values())
        adj = {x: set() for x in nodes}
        for a, b in graph['edges']:
            a, b = tuple(a), tuple(b)
            adj[a].add(b); adj[b].add(a)
        seen = {next(iter(nodes))}
        todo = deque(seen)
        while todo:
            for nxt in adj[todo.popleft()] - seen:
                seen.add(nxt); todo.append(nxt)
        self.assertEqual(seen, nodes)

    def test_paths_are_isolated(self):
        for a in ('supermarket', 'tugbot_warehouse'):
            self.assertNotEqual(scenario_file(WS, a, 2026), scenario_file(WS, 'colored_warehouse', 2026))
            self.assertNotEqual(plans_dir(WS, a, 2026), plans_dir(WS, 'colored_warehouse', 2026))

    def test_scenario_contract_compatible_with_gazebo(self):
        self.assertTrue((WS / 'src/swarm_worlds/worlds/colored_warehouse.sdf').is_file())
        g, o = map_files(WS, 'colored_warehouse')
        _, _, scenario = choose(2026, graph_file=g, geometry_file=o, world='colored_warehouse')
        self.assertEqual((len(scenario['robots']), len(scenario['trash']), len(scenario['deposits'])), (3, 6, 3))
        self.assertEqual(scenario['world'], 'colored_warehouse')
        # Test del mismo contrato de escenario que consume el launch, sin arrancar Gazebo.
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'scene.json'
            path.write_text(json.dumps(scenario), encoding='utf8')
            loaded = load_scenario(path, expected_seed=2026, expected_world='colored_warehouse', geometry_path=o)
            self.assertEqual((len(loaded['robots']), len(loaded['trash']), len(loaded['deposits'])), (3, 6, 3))

if __name__ == '__main__':
    unittest.main()

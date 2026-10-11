import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import json

from escenario_aleatorio_swarm import build, load_scenario
import grafo_basuras_dinamico as dyn
import colocar_basuras_depositos as place

class TestEscenarios(unittest.TestCase):
    def test_reproducible(self):
        a = build(42)
        self.assertEqual(a, build(42))
        self.assertNotEqual(a['elements'], build(2026)['elements'])

    def test_shared_graph(self):
        data = build(42)
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'scene.json'
            p.write_text(json.dumps(data))
            _, elements = load_scenario(p)
            with patch.object(dyn, 'ELEMENTS', elements):
                graph = dyn.DynamicTrashGraph()
                for entry in data['elements']:
                    vertex = tuple(entry['grid'])
                    if entry['type'] == 'trash':
                        self.assertNotIn(vertex, graph.vertices)
                    else:
                        self.assertIn(vertex, graph.vertices)
                self.assertEqual(len(graph.pending), 6)

    def test_reject_modified_xy(self):
        data = build(42)
        data['elements'][0]['xy_m'][0] += 1
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'scene.json'
            p.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_scenario(p)

if __name__ == '__main__':
    unittest.main()

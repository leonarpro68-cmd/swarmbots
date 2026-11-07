"""Pruebas estáticas para rubber_factory, sin ROS ni Gazebo."""
from collections import deque
from pathlib import Path
import math
import sys
import unittest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'scripts'))
sys.path.insert(0, str(WS / 'scripts' / 'supermarket_escenarios'))
from swarm_maps import map_files, validate_map, scenario_file, plans_dir
from generar_escenario import choose

class RubberFactoryTest(unittest.TestCase):
    def test_two_files_and_metadata(self):
        gf, of = map_files(WS, 'rubber_factory')
        self.assertEqual({p.name for p in gf.parent.iterdir()}, {'grafo.json', 'obstaculos.json'})
        g, geo = validate_map(gf, of)
        self.assertEqual((len(g['nodes']), len(g['edges']), len(geo['obstacles'])), (574, 2075, 12))
        self.assertEqual(g['metadata']['resolution_m'], 1.25)
        self.assertEqual(g['metadata']['origin'], [-15.0, -25.0])
        self.assertEqual(geo['bounds'], {'xmin': -14.75, 'xmax':14.75, 'ymin':-24.75, 'ymax':24.75})
        self.assertEqual(geo['graph']['clearance_m'], 0.936)
        self.assertTrue((WS / 'src/swarm_worlds/worlds/rubber_factory.sdf').is_file())

    def test_graph_connected_and_no_duplicate_edges(self):
        g, geo = validate_map(*map_files(WS, 'rubber_factory'))
        nodes = {(round(n['x'],5), round(n['y'],5)) for n in g['nodes']}
        adj = {a:set() for a in nodes}
        distinct=set()
        for a, b in g['edges']:
            a=tuple(a);b=tuple(b)
            self.assertIn(a,nodes);self.assertIn(b,nodes)
            self.assertNotEqual(a,b)
            self.assertLessEqual(abs(a[0]-b[0]),1.25+1e-9)
            self.assertLessEqual(abs(a[1]-b[1]),1.25+1e-9)
            self.assertNotEqual((a[0],a[1]),(b[0],b[1]))
            edge=tuple(sorted((a,b)))
            self.assertNotIn(edge,distinct)
            distinct.add(edge)
            adj[a].add(b);adj[b].add(a)
        seen={min(nodes)}; todo=deque(seen)
        while todo:
            for q in adj[todo.popleft()] - seen:
                seen.add(q);todo.append(q)
        self.assertEqual(seen,nodes)
        # East-of-wall stripe intentionally not navigable (no link from main area).
        self.assertFalse(any(x>12.452 for x,y in nodes))

    def test_node_clearance(self):
        g,geo=validate_map(*map_files(WS,'rubber_factory'))
        m=float(geo['graph']['clearance_m'])
        bounds=geo['bounds']
        for n in g['nodes']:
            x,y=n['x'],n['y']
            self.assertGreaterEqual(min(x-bounds['xmin'],bounds['xmax']-x,y-bounds['ymin'],bounds['ymax']-y),m-1e-8)
            for o in geo['obstacles']:
                d=math.hypot(max(o['xmin']-x,0,x-o['xmax']),max(o['ymin']-y,0,y-o['ymax']))
                self.assertGreaterEqual(d,m-1e-8)

    def test_scenario_and_isolation(self):
        gf,of=map_files(WS,'rubber_factory')
        _,_,sc=choose(2026,graph_file=gf,geometry_file=of,world='rubber_factory')
        self.assertEqual((len(sc['robots']),len(sc['trash']),len(sc['deposits'])),(3,6,3))
        self.assertEqual(sc['world'],'rubber_factory')
        for other in ('supermarket','tugbot_warehouse','colored_warehouse','pallet_warehouse','parking_lot'):
            self.assertNotEqual(scenario_file(WS,'rubber_factory',2026),scenario_file(WS,other,2026))
            self.assertNotEqual(plans_dir(WS,'rubber_factory',2026),plans_dir(WS,other,2026))

if __name__=='__main__':
    unittest.main()

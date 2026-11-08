"""Regresión del mapa electrical_substation (sin ROS/Gazebo)."""
from collections import deque
from pathlib import Path
import sys
import unittest

WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'scripts'))
sys.path.insert(0,str(WS/'scripts'/'supermarket_escenarios'))
from swarm_maps import map_files,validate_map,scenario_file,plans_dir
from generar_escenario import choose

class ElectricalSubstationMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gf,cls.of=map_files(WS,'electrical_substation')
        cls.g,cls.geo=validate_map(cls.gf,cls.of)

    def test_two_files_and_geometry(self):
        self.assertEqual({p.name for p in self.gf.parent.iterdir()},{'grafo.json','obstaculos.json'})
        self.assertEqual((len(self.g['nodes']),len(self.g['edges']),len(self.geo['obstacles'])),(690,2579,19))
        self.assertEqual(self.g['metadata']['resolution_m'],1.25)
        self.assertEqual(self.geo['graph']['origin'],[-15.,-25.])
        self.assertEqual(self.geo['bounds'],{'xmin':-14.75,'xmax':14.75,'ymin':-24.75,'ymax':24.75})
        self.assertEqual(self.geo['graph']['clearance_m'],0.936)
        self.assertTrue((WS/'src/swarm_worlds/worlds/electrical_substation.sdf').is_file())

    def test_connected_unique_edges(self):
        nodes={(n['x'],n['y']) for n in self.g['nodes']}
        adj={n:set() for n in nodes};edges=set()
        for a,b in self.g['edges']:
            a=tuple(a);b=tuple(b)
            self.assertIn(a,nodes);self.assertIn(b,nodes);self.assertNotEqual(a,b)
            self.assertLessEqual(abs(a[0]-b[0]),1.25+1e-9)
            self.assertLessEqual(abs(a[1]-b[1]),1.25+1e-9)
            key=tuple(sorted((a,b)))
            self.assertNotIn(key,edges);edges.add(key)
            adj[a].add(b);adj[b].add(a)
        q=deque([min(nodes)]);seen=set(q)
        while q:
            for nxt in adj[q.popleft()] - seen:
                seen.add(nxt);q.append(nxt)
        self.assertEqual(seen,nodes)
        self.assertEqual(len(edges),2579)
        # Excluidos los ocho nodos del lado aislado de la cerca NE.
        self.assertFalse(any(x>=12.5 and y>=16.25 for x,y in nodes))

    def test_all_nodes_and_edges_respect_clearance(self):
        from shapely.geometry import box, Point, LineString
        clearance=self.geo['graph']['clearance_m']
        b=self.geo['bounds']
        obs=[box(o['xmin'],o['ymin'],o['xmax'],o['ymax']) for o in self.geo['obstacles']]
        for n in self.g['nodes']:
            x,y=n['x'],n['y']
            self.assertGreaterEqual(min(x-b['xmin'],b['xmax']-x,y-b['ymin'],b['ymax']-y),clearance-1e-8)
            p=Point(x,y)
            for rect in obs:
                self.assertGreaterEqual(p.distance(rect),clearance-1e-8)
        for a,z in self.g['edges']:
            line=LineString([a,z])
            for rect in obs:
                self.assertGreaterEqual(line.distance(rect),clearance-1e-8)

    def test_scenario_and_results_isolated(self):
        _,_,sc=choose(2026,graph_file=self.gf,geometry_file=self.of,world='electrical_substation')
        self.assertEqual(sc['world'],'electrical_substation')
        self.assertEqual((len(sc['robots']),len(sc['trash']),len(sc['deposits'])),(3,6,3))
        for other in ('supermarket','tugbot_warehouse','colored_warehouse','pallet_warehouse','parking_lot','rubber_factory'):
            self.assertNotEqual(scenario_file(WS,'electrical_substation',2026),scenario_file(WS,other,2026))
            self.assertNotEqual(plans_dir(WS,'electrical_substation',2026),plans_dir(WS,other,2026))

if __name__=='__main__':unittest.main()

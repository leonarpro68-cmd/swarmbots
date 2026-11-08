"""Pruebas estáticas del mapa pallet_warehouse; no necesitan ROS ni Gazebo."""
import sys
import unittest
from pathlib import Path
from collections import deque
import xml.etree.ElementTree as ET

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'scripts'))
sys.path.insert(0, str(WS / 'scripts' / 'supermarket_escenarios'))
from swarm_maps import map_files, validate_map, scenario_file, plans_dir
from generar_escenario import choose

class PalletWarehouseTest(unittest.TestCase):
    def test_only_two_files_in_map_folder(self):
        graph, obs = map_files(WS, 'pallet_warehouse')
        self.assertEqual({p.name for p in graph.parent.iterdir()}, {'grafo.json', 'obstaculos.json'})
        g, o = validate_map(graph, obs)
        self.assertEqual((len(g['nodes']), len(g['edges']), len(o['obstacles'])), (487,1249,24))
        self.assertEqual(g['metadata']['origin'], [-15.0,-25.0])
        self.assertEqual(o['bounds'], dict(xmin=-15.0,xmax=15.0,ymin=-25.0,ymax=25.0))

    def test_graph_is_connected(self):
        g,_ = validate_map(*map_files(WS,'pallet_warehouse'))
        nodes = {(round(n['x'],5),round(n['y'],5)) for n in g['nodes']}
        adj = {n:set() for n in nodes}
        for aa,bb in g['edges']:
            a,b = tuple(aa),tuple(bb)
            adj[a].add(b);adj[b].add(a)
        seen={min(nodes)}
        queue=deque(seen)
        while queue:
            for n in adj[queue.popleft()]-seen:
                seen.add(n);queue.append(n)
        self.assertEqual(nodes,seen)

    def test_source_world_and_independent_paths(self):
        world=WS/'src/swarm_worlds/worlds/pallet_warehouse.sdf'
        self.assertTrue(world.is_file())
        collisions=ET.parse(world).getroot().findall('.//model[@name="pallet_warehouse"]/link/collision')
        self.assertEqual(len(collisions),29)  # 24 interiores + suelo + 4 paredes
        for m in ('supermarket','tugbot_warehouse','colored_warehouse'):
            self.assertNotEqual(scenario_file(WS,'pallet_warehouse',2026),scenario_file(WS,m,2026))
            self.assertNotEqual(plans_dir(WS,'pallet_warehouse',2026),plans_dir(WS,m,2026))

    def test_scenario_generator(self):
        g,o=map_files(WS,'pallet_warehouse')
        _,_,scene=choose(2026,graph_file=g,geometry_file=o,world='pallet_warehouse')
        self.assertEqual((len(scene['robots']),len(scene['trash']),len(scene['deposits'])),(3,6,3))
        self.assertEqual(scene['world'],'pallet_warehouse')

if __name__=='__main__': unittest.main()

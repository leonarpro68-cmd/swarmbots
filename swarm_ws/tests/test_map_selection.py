"""Pruebas de contrato de selección de mapas, sin ROS 2/Gazebo."""
import json
import sys
import unittest
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS/'scripts'))
sys.path.insert(0, str(WS/'scripts/supermarket_escenarios'))
from swarm_maps import (MAPS, map_files, scenario_file, generation_dir,
                        milp_results_dir, plans_dir, validate_map)
from generar_escenario import choose


class MapSelectionTest(unittest.TestCase):
    def test_only_two_files_per_map(self):
        for map_name in MAPS:
            folder=WS/'config/maps'/map_name
            self.assertEqual({x.name for x in folder.iterdir()}, {'grafo.json','obstaculos.json'})
            graph,geometry=map_files(WS,map_name)
            validate_map(graph,geometry)

    def test_tugbot_topology(self):
        graph,_=map_files(WS,'tugbot_warehouse')
        data=json.loads(graph.read_text())
        self.assertEqual((len(data['nodes']),len(data['edges'])),(767,2816))

    def test_paths_dont_collide_across_maps(self):
        seed=2026
        for fn in (scenario_file,generation_dir,milp_results_dir,plans_dir):
            self.assertNotEqual(fn(WS,'supermarket',seed), fn(WS,'tugbot_warehouse',seed))
        self.assertEqual(scenario_file(WS,'supermarket',seed), WS/'config/supermarket/seed_2026.json')

    def test_tugbot_scenario_is_deterministic(self):
        g,o=map_files(WS,'tugbot_warehouse')
        _,_,a=choose(2026,graph_file=g,geometry_file=o,world='tugbot_warehouse')
        _,_,b=choose(2026,graph_file=g,geometry_file=o,world='tugbot_warehouse')
        self.assertEqual(a,b)
        self.assertEqual((len(a['robots']),len(a['trash']),len(a['deposits'])), (3,6,3))
        self.assertEqual(a['world'],'tugbot_warehouse')

    def test_supermarket_original_rectangle_is_retained(self):
        g,o=map_files(WS,'supermarket')
        _,_,scene=choose(2026,graph_file=g,geometry_file=o)
        self.assertEqual(scene['trash_spawn_rectangle'],
                         {'xmin':-13.75,'xmax':13.75,'ymin':-23.75,'ymax':22.5})
        self.assertEqual(scene['world'],'supermarket')


if __name__=='__main__':unittest.main()

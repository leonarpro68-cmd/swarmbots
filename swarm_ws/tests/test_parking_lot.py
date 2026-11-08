"""Pruebas estáticas de parking_lot (sin ROS ni Gazebo)."""
from collections import deque
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / 'scripts'))
sys.path.insert(0, str(WS / 'scripts' / 'supermarket_escenarios'))
from swarm_maps import map_files, validate_map, scenario_file, plans_dir
from generar_escenario import choose

class ParkingLotTest(unittest.TestCase):
    def test_only_two_files_and_geometry(self):
        gf,of=map_files(WS,'parking_lot')
        self.assertEqual({p.name for p in gf.parent.iterdir()}, {'grafo.json','obstaculos.json'})
        graph,geo=validate_map(gf,of)
        self.assertEqual((len(graph['nodes']),len(graph['edges']),len(geo['obstacles'])), (419,1220,53))
        self.assertEqual(graph['metadata']['origin'],[-15.0,-25.0])
        self.assertEqual(geo['bounds'], {'xmin':-15.0,'xmax':15.0,'ymin':-25.0,'ymax':25.0})
        self.assertEqual(graph['metadata']['resolution_m'],1.25)

    def test_connected_and_no_duplicate_edges(self):
        graph,_=validate_map(*map_files(WS,'parking_lot'))
        nodes={tuple((round(n['x'],5),round(n['y'],5))) for n in graph['nodes']}
        adj={n:set() for n in nodes}
        edges=set()
        for aa,bb in graph['edges']:
            a,b=tuple(aa),tuple(bb)
            self.assertIn(a,nodes);self.assertIn(b,nodes)
            self.assertNotEqual(a,b)
            edge=tuple(sorted((a,b)))
            self.assertNotIn(edge,edges)
            edges.add(edge)
            self.assertLessEqual(abs(a[0]-b[0]),1.25+1e-8)
            self.assertLessEqual(abs(a[1]-b[1]),1.25+1e-8)
            adj[a].add(b);adj[b].add(a)
        seen={min(nodes)};todo=deque(seen)
        while todo:
            for nxt in adj[todo.popleft()]-seen:
                seen.add(nxt);todo.append(nxt)
        self.assertEqual(seen,nodes)

    def test_exact_source_collision_boxes(self):
        sdf=WS/'src/swarm_worlds/worlds/parking_lot.sdf'
        collisions=ET.parse(sdf).getroot().findall('.//model[@name="parking_lot"]/link/collision')
        self.assertEqual(len(collisions),58)
        _,geo=validate_map(*map_files(WS,'parking_lot'))
        obs={x['name']:x for x in geo['obstacles']}
        self.assertEqual(len(obs),53)
        for col in collisions:
            name=col.get('name')
            if name=='floor_c' or name.startswith('wall_'):continue
            key=name.removesuffix('_c')
            self.assertIn(key,obs)
            x,y,*_=map(float,col.findtext('pose').split())
            size=col.findtext('geometry/box/size')
            if size:
                sx,sy,*_=map(float,size.split());rx,ry=sx/2,sy/2
            else:
                rx=ry=float(col.findtext('geometry/cylinder/radius'))
            expected={'name':key,'xmin':round(x-rx,5),'xmax':round(x+rx,5),'ymin':round(y-ry,5),'ymax':round(y+ry,5)}
            self.assertEqual(obs[key],expected)

    def test_scenarios_are_independent(self):
        g,o=map_files(WS,'parking_lot')
        _,_,sc=choose(2026,graph_file=g,geometry_file=o,world='parking_lot')
        self.assertEqual((len(sc['robots']),len(sc['trash']),len(sc['deposits'])), (3,6,3))
        self.assertEqual(sc['world'],'parking_lot')
        for m in ('supermarket','tugbot_warehouse','colored_warehouse','pallet_warehouse'):
            self.assertNotEqual(scenario_file(WS,'parking_lot',2026),scenario_file(WS,m,2026))
            self.assertNotEqual(plans_dir(WS,'parking_lot',2026),plans_dir(WS,m,2026))

if __name__=='__main__':unittest.main()

"""Adaptador aislado para el grafo de supermarket. No altera geometría original."""
import json, math
from pathlib import Path
STEP_M=1.25
ORIGIN_X=-15.0
ORIGIN_Y=-25.0
ROBOT_LENGTH_M=0.7190
ROBOT_WIDTH_M=0.5786
ROBOT_RADIUS_M=math.hypot(ROBOT_LENGTH_M, ROBOT_WIDTH_M)/2
EXTRA_MARGIN_M=0.30
ROOT=Path(__file__).resolve().parents[1]

def node_position(v):
    return (ORIGIN_X + v[0]*STEP_M,ORIGIN_Y+v[1]*STEP_M)

def make_graph():
    data=json.loads((ROOT/'supermarket_grafo.json').read_text())
    vertices={(int(n['i']),int(n['j'])) for n in data['nodes']}
    xy_to_node={(round(n['x'],5),round(n['y'],5)):(int(n['i']),int(n['j'])) for n in data['nodes']}
    edges=set()
    for a,b in data['edges']:
        x=xy_to_node[(round(a[0],5),round(a[1],5))]
        y=xy_to_node[(round(b[0],5),round(b[1],5))]
        edges.add(tuple(sorted((x,y))))
    return vertices,edges,None,None

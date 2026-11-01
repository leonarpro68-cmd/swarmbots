"""Recupera el contrato del assignment.approach_vertices() y
PlanningBridge.deposit_goals() de SwarmBots.

Diferencia necesaria: en supermarket las basuritas ocupan coordenadas
continuas, así que se utiliza la celda/nodo lattice más próximo como
ANCLA de una vecindad de Moore de 8 nodos. La ancla NO es una meta.

No implica por sí mismo que el mecanismo de recogida tenga alcance físico.
"""
from __future__ import annotations
from . import geometry


def anchor_node(xy):
    """Índices del nodo geométricamente más próximo en la retícula completa."""
    return (round((float(xy[0])-geometry.ORIGIN_X)/geometry.STEP_M),
            round((float(xy[1])-geometry.ORIGIN_Y)/geometry.STEP_M))


def neighbor_vertices(graph, anchor):
    i,j = anchor
    return tuple(sorted((i+di,j+dj) for di in (-1,0,1) for dj in (-1,0,1)
                        if (di or dj) and (i+di,j+dj) in graph.vertices))


def approach_vertices(graph, trash_name):
    if trash_name not in graph.trash:
        raise ValueError(f'Basura desconocida: {trash_name}')
    return neighbor_vertices(graph, anchor_node(graph.trash[trash_name]['xy_m']))


def deposit_goals(graph, color):
    deposits=[d for d in graph.deposits.values() if d['color']==color]
    if len(deposits)!=1:
        raise ValueError(f'Se requiere depósito único de color {color}')
    return neighbor_vertices(graph, tuple(deposits[0]['grid']))


def inspect_targets(graph, scenario):
    return {
        'trash':{t['name']: {
            'anchor':list(anchor_node(t['xy_m'])),
            'candidates':[list(n) for n in approach_vertices(graph,t['name'])],
            'distance_m':[round(__import__('math').dist(geometry.node_position(n),t['xy_m']),4)
                          for n in approach_vertices(graph,t['name'])]
        } for t in scenario['trash']},
        'deposits':{d['name']: {
            'center_node':d['grid'],
            'candidates':[list(n) for n in deposit_goals(graph,d['color'])]
        } for d in scenario['deposits']}
    }

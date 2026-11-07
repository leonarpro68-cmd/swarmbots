#!/usr/bin/env python3
"""Deterministic FIXED baseline: política de asignación FIXED seed100;
first/second garbage by sorted name. No MILP optimization is performed."""
import argparse,json
from pathlib import Path

def generate(scene):
    if scene.get('world')!='supermarket' or not isinstance(scene.get('seed'),int):
        raise ValueError('Escenario incompatible')
    robots=sorted(r['name'] for r in scene['robots'])
    if robots!=['summit0','summit1','summit2']:raise ValueError('Se requieren exactamente tres Summit')
    by_color={c:sorted(t['name'] for t in scene['trash'] if t['color']==c) for c in ('verde','azul','naranja')}
    if any(len(v)!=2 for v in by_color.values()):raise ValueError('FIXED requiere exactamente dos basuras por color')
    if set('deposit_'+c for c in by_color)!=set(d['name'] for d in scene['deposits']):
        raise ValueError('Depósitos faltantes')
    # Reuse the canonical FIXED task allocation of the seed100 experiment.
    # Names are stable across scenarios; locations vary by seed.
    pickups={
        'summit0':('trash_A2_azul','trash_A1_azul'),
        'summit1':('trash_V2_verde','trash_N2_naranja'),
        'summit2':('trash_N1_naranja','trash_V1_verde'),
    }
    all_trash={t['name']:t for t in scene['trash']}
    if set(n for pair in pickups.values() for n in pair)!=set(all_trash):
        raise ValueError('Inventario diferente del FIXED seed100; no inventar asignación')
    chosen={}
    for robot,(first,second) in pickups.items():
        c1=all_trash[first]['color'];c2=all_trash[second]['color']
        chosen[robot]={'pickup1':first,'delivery1':'deposit_'+c1,'pickup2':second,'delivery2':'deposit_'+c2}
    return {'schema':'supermarket.fixed.assignment.offline.v1','status':'FIXED_DETERMINISTIC_OFFLINE_NOT_EXECUTABLE',
       'execution_authorized':False,'world':'supermarket','seed':scene['seed'],'chosen':chosen,
       'policy':'política de asignación FIXED seed100; sorted trash IDs per round; no MILP',
       'note':'Baseline determinista; puede ser igual a MILP en ciertos escenarios'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--scenario',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():ap.error('No se sobrescribe la asignación FIXED existente')
    data=generate(json.loads(a.scenario.read_text()))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
    print('FIXED_OFFLINE_ASSIGNMENT:',a.output)
if __name__=='__main__':main()

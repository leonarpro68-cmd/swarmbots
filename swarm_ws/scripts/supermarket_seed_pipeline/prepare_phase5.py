#!/usr/bin/env python3
"""Fase 5: bridge solo OFFLINE para MILP/CBS/Hybrid/V3 -> contratos D8.

No publica mensajes ROS, no mueve robots, no autoriza ejecución física.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import map_files

import numpy as np
from shapely.geometry import LineString, Point, box

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=tuple(f'summit{i}' for i in range(3))
ROBOT_RADIUS=0.55
SHELF_CENTER_CLEARANCE=0.60
OTHER_TRASH_EXCLUSION=0.55+math.sqrt(2)*0.09/2+0.05
OTHER_DEPOSIT_EXCLUSION=0.50
PICKUP_D8_MIN=0.55
PICKUP_D8_MAX=2.20
DELIVERY_D8_MIN=0.90
DELIVERY_D8_MAX=1.80
PICKUP_STOP_CENTER_DISTANCE=0.65  # solo inspección geométrica conservadora, NO distancia contractual D8
DELIVERY_STOP_CENTER_DISTANCE=0.80


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_new(path, data):
    path=Path(path)
    if path.exists(): raise FileExistsError(f'No sobrescribir: {path}')
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def xy(grid, geo):
    g=geo['graph']
    return [round(float(g['origin'][k])+float(g['step_m'])*grid[k],8) for k in (0,1)]


def cbs_stage(cbs, name):
    stages=cbs['stages']
    if isinstance(stages,dict):
        raw=stages[name]
        return raw
    raise ValueError('Formato CBS inesperado (se esperaba stages como diccionario)')


def audit_local(start, center, stage, target, pending, scene, geom):
    d=math.dist(start,center)
    stop=PICKUP_STOP_CENTER_DISTANCE if stage.startswith('pickup') else DELIVERY_STOP_CENTER_DISTANCE
    # Ruta de intención para diagnosticar un acercamiento frontal; D8 decide su movimiento real.
    end=[center[i]+(start[i]-center[i])*stop/d for i in (0,1)]
    segment=LineString([start,end])
    shelf=[]
    for o in geom['obstacles']:
        ds=segment.distance(box(o['xmin'],o['ymin'],o['xmax'],o['ymax']))
        if ds < SHELF_CENTER_CLEARANCE-1e-6:shelf.append({'name':o['name'],'distance_m':round(ds,4)})
    other_trash=[]
    for item in scene['trash']:
        if item['name']==target or item['name'] not in pending:continue
        ds=segment.distance(Point(item['xy_m']))
        if ds < OTHER_TRASH_EXCLUSION-1e-6:
            other_trash.append({'name':item['name'],'distance_m':round(ds,4)})
    deposits=[]
    for item in scene['deposits']:
        if item['name']==target:continue
        ds=segment.distance(Point(item['xy_m']))
        if ds < OTHER_DEPOSIT_EXCLUSION-1e-6:
            deposits.append({'name':item['name'],'distance_m':round(ds,4)})
    return {'inspect_standoff_m':stop, 'inspection_endpoint_xy_m':[round(v,6) for v in end],
            'shelf_hits':shelf,'other_pending_trash_hits':other_trash,'other_deposit_hits':deposits,
            'straight_line_geometric_screen_pass':not(shelf or other_trash or deposits),
            'note':'Solo línea ideal; NO demuestra visibilidad RGB-D, estabilidad de carga ni ausencia de colisión en Gazebo'}


def prepare(cbs,scene,geom,results,orig_cbs,orig_scenario,orig_geom):
    errors=[]
    if not isinstance(scene.get('world'),str) or not scene.get('world') or not isinstance(scene.get('seed'),int) or scene['seed']<0:errors.append('Mapa o semilla del escenario inválidos')
    if (cbs.get('seed'),cbs.get('status'),cbs.get('execution_authorized'))!=(scene.get('seed'),'OFFLINE_CBS_ONLY_NOT_EXECUTABLE',False):
        errors.append('Estado CBS incompatible o no offline')
    if list(cbs.get('stage_sequence',[]))!=list(STAGES):errors.append('Secuencia de etapas incorrecta')
    if cbs.get('inputs_sha256',{}).get('scenario') != sha(orig_scenario):errors.append('SHA escenario CBS no corresponde al archivo real')
    if len(scene['trash'])!=6 or len(scene['deposits'])!=3 or len(scene['robots'])!=3:errors.append('Inventario de escenario incorrecto')
    item={o['name']:o for o in scene['trash']+scene['deposits']}
    config=cbs['assignment']
    picks=[]
    for r in ROBOTS:
        for stage in ('pickup1','pickup2'):
            t=config[r][stage]
            picks.append(t)
            if t not in item or item[t].get('type')!='trash':errors.append(f'{r}/{stage}: no es basura real')
            else:
                want='deposit_'+item[t]['color']
                if config[r]['delivery'+stage[-1]]!=want:errors.append(f'{r}: depósito de color incorrecto')
    if len(set(picks))!=6:errors.append('Basuritas asignadas repetidas u omitidas')
    previous=None; nav={}; assignments={};rows_all=[]; nav_hashes={}
    for stage in STAGES:
        st=cbs_stage(cbs,stage)
        paths=st.get('paths',[])
        if len(paths)!=3:errors.append(f'{stage}: no hay 3 caminos')
        goals=st['goals_grid']
        if len(goals)!=3:errors.append(f'{stage}: no hay 3 metas')
        stage_starts=[xy(p['grid'][0],geom) for p in paths]
        stage_ends=[xy(p['grid'][-1],geom) for p in paths]
        for i,p in enumerate(paths):
            if list(p['grid'][0])!=list(st['starts_grid'][i]):errors.append(f'{stage}: CBS inicio inconsistente r{i}')
            if list(p['grid'][-1])!=list(st['goals_grid'][i]):errors.append(f'{stage}: CBS fin inconsistente r{i}')
        if previous is not None:
            if any(math.dist(a,b)>1e-6 for a,b in zip(previous,stage_starts)):errors.append(f'{stage}: discontinuidad con etapa anterior')
        else:
            for i,a in enumerate(stage_starts):
                if math.dist(a,scene['robots'][i]['xy_m'])>1e-6:errors.append(f'{stage}: origen de summit{i} no coincide')
        previous=stage_ends
        folder=Path(results)/stage
        npz=folder/f'{stage}_MILP_TIMED_OFFLINE.npz'
        clockpath=folder/f'{stage}_MILP_V3_clock.json'
        reportpath=folder/f'{stage}_report.json'
        if not all(p.is_file() for p in [npz,clockpath,reportpath]):
            errors.append(f'{stage}: faltan NPZ/reloj/reporte');continue
        rep=read(reportpath);clock=read(clockpath)
        digest=sha(npz);nav_hashes[stage]=digest
        if rep.get('stage')!=stage or rep.get('status')!='PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE' or rep.get('refinements_accepted')!=5 or not rep.get('continuous_certificate',{}).get('pass_certificate'):
            errors.append(f'{stage}: certificado Hybrid V2 no apto')
        if rep.get('execution_authorized') is not False or clock.get('execution_authorized') is not False:
            errors.append(f'{stage}: salida offline inconsistente')
        if clock.get('status')!='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE' or clock.get('common_time_warp') is not True or clock.get('input_sha256')!=digest:
            errors.append(f'{stage}: reloj V3 desacoplado del NPZ')
        if clock.get('max_speed_m_s')!=2.0 or clock.get('max_accel_m_s2')!=2.0:
            errors.append(f'{stage}: límites V3 inesperados')
        if rep.get('targets')!=st.get('targets'):
            errors.append(f'{stage}: destinos report Hybrid no son CBS')
        try:
            with np.load(npz,allow_pickle=False) as z:
                for i,r in enumerate(ROBOTS):
                    c=z[f'R{i}_controls'];ds=z[f'R{i}_durations_s'];
                    if c.ndim!=3 or c.shape[1:]!=(6,2) or ds.shape!=(len(c),) or not np.isfinite(c).all() or not np.all(np.isfinite(ds)&(ds>0)):
                        errors.append(f'{stage}/{r}: arrays inválidos');continue
                    if math.dist(c[0,0],stage_starts[i])>1e-5 or math.dist(c[-1,-1],stage_ends[i])>1e-5:
                        errors.append(f'{stage}/{r}: NPZ no coincide con CBS')
        except Exception as exc:errors.append(f'{stage}: fallo NPZ {exc}')
        robots={}
        for i,r in enumerate(ROBOTS):
            target=st['targets'][i]
            if config[r][stage]!=target:errors.append(f'{stage}/{r}: target CBS ≠ MILP')
            center=item[target]['xy_m']
            dist=math.dist(stage_ends[i],center)
            low,high=(PICKUP_D8_MIN,PICKUP_D8_MAX) if stage.startswith('pickup') else (DELIVERY_D8_MIN,DELIVERY_D8_MAX)
            range_ok=(low+1e-6<dist<high-1e-6)
            if not range_ok:errors.append(f'{stage}/{r}: distancia D8 {dist:.3f} fuera de ({low},{high})')
            geom_audit=audit_local(stage_ends[i],center,stage,target,st['pending_before'],scene,geom)
            if not geom_audit['straight_line_geometric_screen_pass']:
                errors.append(f'{stage}/{r}: posible choque en tramo D8 local')
            entry={'robot':r,'target':target,'color':item[target]['color'],'approach_xy_m':stage_ends[i],
                   'target_xy_m':center,'distance_m':round(dist,6),'d8_distance_min_m':low,'d8_distance_max_m':high,
                   'd8_distance_gate':range_ok,'distance_to_max_limit_m':round(high-dist,6),
                   'local_corridor_screen':geom_audit,'d8_camera_and_grip_verified':False}
            rows_all.append(dict(stage=stage,**entry))
            robots[r]={'trash':config[r]['pickup'+stage[-1]],'color':item[config[r]['pickup'+stage[-1]]]['color'],
                       'goal_xy_m':stage_ends[i], 'target':target,'target_xy_m':center,
                       'distance_m':round(dist,6)}
        assignments[stage]={'schema':'supermarket.milp.physical.assignment.offline.v1','world':scene['world'],'seed':scene['seed'],
                            'stage':stage,'navigation_sha256':digest,'robots':robots,
                            'execution_authorized':False}
        nav[stage]={'stage':stage,'trajectory':str(npz.absolute()),'successive_clock':str(clockpath.absolute()),
                    'sha256':digest,'start_xy_m':stage_starts,'goal_xy_m':stage_ends,
                    'duration_v3_s':round(float(clock['duration_successive_s']),6)}
    report={'schema':'supermarket.milp.phase5.d8_nav_gate.v1',
            'status':'PASS_STATIC_PRECHECK_NOT_EXECUTABLE' if not errors else 'REJECTED_STATIC_PRECHECK',
            'execution_authorized':False,'physical_D8_reach_verified':False,
            'sha256':{'cbs':sha(orig_cbs),'scenario':sha(orig_scenario),'geometry':sha(orig_geom)},
            'stages':nav,'d8_tasks':rows_all,'errors':errors,
            'limitations':['Aproximación rectilínea aproximada, no trayectoria ejecutada de visual_grasp',
                           'D8 no garantiza distancia real hasta verificar odometría',
                           'Sin inspección de RGB-D, orientación, gripper, joint ni descarga en Gazebo',
                           'No se ha emitido ningún comando ROS']}
    return report,assignments


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assignment-mode',choices=('fixed','milp'),default='milp')
    p.add_argument('--cbs',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--geometry',type=Path,default=None)
    p.add_argument('--results',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    if a.output_dir.exists() and any(a.output_dir.iterdir()):p.error('Directorio de salida no vacío, no sobrescribir')
    geo=a.geometry or map_files(Path(__file__).resolve().parents[2], 'supermarket')[1]
    cb=read(a.cbs)
    if cb.get('assignment_mode', 'milp')!=a.assignment_mode:raise ValueError('Modo CBS incompatible con modo solicitado')
    rep,assignments=prepare(cb,read(a.scenario),read(geo),a.results,a.cbs,a.scenario,geo)
    a.output_dir.mkdir(parents=True,exist_ok=True)
    write_new(a.output_dir/'phase5_static_preflight.json',rep)
    if not rep['errors']:
        sc=read(a.scenario)
        sc['elements']=sc['trash']+sc['deposits']
        sc['milp_origin']='assignment from CBS multigoal, not static preassigned mission'
        suffix=a.assignment_mode.upper()
        write_new(a.output_dir/f"scenario_for_original_D8_seed{sc['seed']}_{suffix}.json",sc)
        for stage,data in assignments.items():
            write_new(a.output_dir/f'assignment_{stage}_{suffix}.json',data)
        write_new(a.output_dir/'mission_navigation_contract.json',{'schema':'supermarket.milp.phase5.navigation.contract.v1',
               'execution_authorized':False,'stages':rep['stages'],'source_sha256':rep['sha256']})
    print(rep['status'])
    for row in rep['d8_tasks']:
        print(f" {row['stage']:10} {row['robot']:8} {row['target']:23} d={row['distance_m']:.3f}m; slack={row['distance_to_max_limit_m']:.3f}m; corridor={'OK' if row['local_corridor_screen']['straight_line_geometric_screen_pass'] else 'FAIL'}")
    for err in rep['errors']:print(' ERROR:',err)
    print('Sin órdenes ROS; NO autoriza ejecución física.')
    return 0 if not rep['errors'] else 2

if __name__=='__main__':sys.exit(main())

#!/usr/bin/env python3
"""D9.9.20-S: retorno seguro del Summit 0 tras delivery2 wave2.

La meta es su posición medida después de wave1 (retorno local ya efectuado),
NO la posición de parking ni una pose inventada. Reutiliza planificador CBS
y el path_follower existentes; otros dos Summit son obstáculos fijos.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROBOTS = ('summit0','summit1','summit2')

def save(p, d):
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(d, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    os.replace(tmp,p)

def snapshot():
    from d6_grasp_existing import snapshot_odom
    raw=snapshot_odom()
    xy={}; speed={}
    for r in ROBOTS:
        d=raw[r]
        pos=d.get('xy',d.get('xy_m'))
        if pos is None: raise RuntimeError(f'{r}: posición no disponible')
        xy[r]=tuple(map(float,pos))
        speed[r]=float(d.get('speed_m_s',d.get('speed',0.0)))
    return xy,speed

def plan(start, others, goal):
    from d9920r_return_to_pickup2 import plan_cbs
    # Mismo CBS y clearance que ya usa la transición R.
    route, destination, start_snap, goal_snap=plan_cbs(start, others, goal)
    return route,destination,start_snap,goal_snap

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wave1-report',type=Path,required=True)
    ap.add_argument('--wave2-report',type=Path,required=True)
    ap.add_argument('--clearance-report',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--timeout-s',type=float,default=120)
    a=ap.parse_args()
    d={'schema':'swarm.d9920s.final_return.v1','status':'REJECT_STOPPED',
       'commands_may_have_been_sent':False,'problems':[],'stage':'delivery2'}
    try:
        w1=json.loads(a.wave1_report.read_text())
        w2=json.loads(a.wave2_report.read_text())
        clr=json.loads(a.clearance_report.read_text())
        for name,obj,w in [('wave1',w1,1),('wave2',w2,2)]:
            if obj.get('stage')!='delivery2' or obj.get('wave_requested')!=w or obj.get('status')!='WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION':
                raise RuntimeError(f'{name}: reporte de oleada no acreditado')
        if clr.get('stage')!='delivery2' or clr.get('status')!='DEPOSIT_CLEAR_VERIFIED':
            raise RuntimeError('Despeje de delivery2 no acreditado')
        if 'summit0' not in w1.get('completed_robots',[]) or 'summit1' not in w2.get('completed_robots',[]):
            raise RuntimeError('Faltan confirmaciones de ambas oleadas')
        # La posición posterior a wave1 es la pose después del regreso local.
        target=w1.get('poses_live',{}).get('summit0')
        if not isinstance(target,(list,tuple)) or len(target)!=2:
            raise RuntimeError('Wave1 no guarda poses_live.summit0; prohibido inventar destino')
        target=tuple(map(float,target))
        xy,speed=snapshot()
        if any(v>.06 for v in speed.values()):
            raise RuntimeError(f'Robots aún en movimiento: {speed}')
        if math.dist(xy['summit0'],clr['parking_xy_m'])>.30:
            raise RuntimeError('Summit0 no está en el parking certificado')
        others={r:xy[r] for r in ('summit1','summit2')}
        if any(math.dist(target,p)<1.55 for p in others.values()):
            raise RuntimeError('La pose final de Summit0 está demasiado cerca de un Summit estacionado')
        route,goal,ss,sg=plan(xy['summit0'],others,target)
        d.update({'target_xy_m':goal,'route_xy_m':route,'initial_odometry':xy,
                  'start_snap_m':ss,'goal_snap_m':sg})
        if not a.execute:
            d['status']='FINAL_RETURN_PLANNED_NO_COMMANDS'
        else:
            from d9920t_timed_aux import execute_route
            d['status']='FINAL_RETURN_IN_PROGRESS'
            save(a.output,d)
            d['commands_may_have_been_sent']=True
            save(a.output,d)
            details=execute_route(route,output_report=a.output,timeout_s=a.timeout_s)
            d.update(details)
            save(a.output,d)
            final,sp=snapshot()
            d['final_odometry']=final;d['final_speed_m_s']=sp
            if math.dist(final['summit0'],goal)>.12 or any(x>.08 for x in sp.values()):
                raise RuntimeError('No se ha verificado la pose final o el reposo')
            d['status']='FINAL_RETURN_VERIFIED'
    except Exception as exc:
        d['status']='REJECT_STOPPED';d['problems'].append(f'{type(exc).__name__}: {exc}')
    save(a.output,d)
    print(json.dumps({'status':d['status'],'target_xy_m':d.get('target_xy_m'),
                      'problems':d['problems'],'output':str(a.output)},ensure_ascii=False))
    return 0 if d['status'] in ('FINAL_RETURN_PLANNED_NO_COMMANDS','FINAL_RETURN_VERIFIED') else 2

if __name__=='__main__':sys.exit(main())

#!/usr/bin/env python3
"""D9.9.23: experimento OFFLINE de asignación MILP + CBS + Bézier, NO ejecutable en Gazebo.

El MILP elige el menor tiempo entre asignaciones candidatas con las cuatro etapas
certificadas por Hybrid V2. No afirma optimalidad global sobre las 720 asignaciones.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import numpy as np
from scipy.optimize import milp, Bounds, LinearConstraint
from swarm_behavior.swarm_planning.bridge import PlanningBridge, PickupPlan, DeliveryPlan
from swarm_behavior.swarm_planning.assignment import approach_vertices, solve_multi, assign_nearest
from swarm_behavior.swarm_planning.scenario import load
from swarm_behavior.swarm_planning.geometry import node_position
from swarm_behavior.swarm_planning.hybrid_v2.cbs_adapter import plan_v2

STAGES=('pickup1','delivery1','pickup2','delivery2')

def base_assignment(scenario):
    bridge=PlanningBridge(scenario)
    starts=tuple(map(tuple,scenario.robots))
    # La segunda ronda original depende de posiciones posteriores a CBS; se obtiene
    # con la fachada de planificación, sin modificar el escenario principal.
    bridge=PlanningBridge(scenario)
    p=bridge.plan_pickup(starts)
    one=tuple(x[0] for x in p.assigned)
    for x in one:bridge.confirm_collected(x)
    d=bridge.plan_delivery(tuple(path[-1] for path in p.paths),one)
    q=bridge.plan_pickup(tuple(path[-1] for path in d.paths))
    return one,tuple(x[0] for x in q.assigned)

def proxy_cost(scenario,pair):
    """Solo priorización de candidatos: no se presenta como tiempo Bézier."""
    bridge=PlanningBridge(scenario)
    starts=tuple(scenario.robots)
    total=0.
    for k in (0,1):
        pickup=[]; delivery=[]
        for r in range(3):
            name=pair[k][r]
            pickup_nodes=approach_vertices(bridge.graph,name)
            if not pickup_nodes:return math.inf
            p=min(pickup_nodes,key=lambda v: math.dist(node_position(starts[r]),node_position(v)))
            color=bridge.graph.trash[name].color
            depot_nodes=bridge.deposit_goals(color)
            d=min(depot_nodes,key=lambda v:math.dist(node_position(p),node_position(v)))
            pickup.append(math.dist(node_position(starts[r]),node_position(p)))
            delivery.append(math.dist(node_position(p),node_position(d)))
        colors=[bridge.graph.trash[name].color for name in pair[k]]
        total+=max(pickup)+max(delivery)+15.98*max(colors.count(c) for c in set(colors))
        for name in pair[k]:bridge.confirm_collected(name)
        # El proxy no presume nodos finales reales; la evaluación posterior sí.
        starts=tuple(min(bridge.deposit_goals(bridge.graph.trash[pair[k][r]].color),
                 key=lambda v:math.dist(node_position(v),node_position(starts[r]))) for r in range(3))
    return total

def fixed_pickup(bridge,starts,names,args):
    goals=tuple(approach_vertices(bridge.graph,n) for n in names)
    if any(not g for g in goals):raise ValueError('Basura sin nodo de aproximación')
    res=solve_multi(bridge.graph,starts,goals,max_ct=args.max_ct,max_time=args.max_time)
    return PickupPlan(tuple(starts),tuple((n,0.) for n in names),goals,
                      res['paths'],res['cost'],res['expanded'],res['generated'])

def fixed_delivery(bridge,starts,names,args):
    colors=tuple(bridge.graph.trash[n].color for n in names)
    goals=tuple(bridge.deposit_goals(c) for c in colors)
    res=solve_multi(bridge.graph,starts,goals,max_ct=args.max_ct,max_time=args.max_time)
    return DeliveryPlan(tuple(starts),tuple(names),colors,goals,
                        res['paths'],res['cost'],res['expanded'],res['generated'])

def evaluate(scenario,pair,args):
    bridge=PlanningBridge(scenario)
    starts=tuple(map(tuple,scenario.robots))
    steps={};T=0.;waves=[]
    for k in range(2):
        names=tuple(pair[k]);s=f'pickup{k+1}'
        p=fixed_pickup(bridge,starts,names,args)
        rep,_=plan_v2(p,bridge.graph,stage=s,margin=args.margin,
                      iterations=args.iterations,max_speed=args.max_speed,
                      max_accel=args.max_accel,maxiter=args.maxiter)
        if not rep.get('certified'):raise RuntimeError(f'{s}: {rep.get("reason",rep.get("status"))}')
        nav=float(rep['duration_s']);T+=nav+args.pickup_s
        steps[s]={'duration_bezier_s':nav,'physical_wave_s':args.pickup_s,'status':'CERTIFIED','names':list(names)}
        starts=tuple(path[-1] for path in p.paths)
        for name in names:bridge.confirm_collected(name)
        s=f'delivery{k+1}'
        d=fixed_delivery(bridge,starts,names,args)
        rep,_=plan_v2(d,bridge.graph,stage=s,margin=args.margin,
                      iterations=args.iterations,max_speed=args.max_speed,
                      max_accel=args.max_accel,maxiter=args.maxiter)
        if not rep.get('certified'):raise RuntimeError(f'{s}: {rep.get("reason",rep.get("status"))}')
        nav=float(rep['duration_s'])
        count=max(d.colors.count(c) for c in set(d.colors))
        # Este presupuesto añade operaciones físicas después de la navegación
        # de etapa. La lógica real de navegación/retorno entre oleadas requiere
        # verificación adicional para equivalencia exacta con Gazebo.
        T+=nav+count*args.delivery_s;waves.append(count)
        steps[s]={'duration_bezier_s':nav,'physical_waves':count,'physical_wave_s':args.delivery_s,
                  'status':'CERTIFIED','colors':list(d.colors)}
        starts=tuple(path[-1] for path in d.paths)
    return {'pickup1':list(pair[0]),'pickup2':list(pair[1]),'waves_delivery':waves,
            'navigation_s':sum(v['duration_bezier_s'] for v in steps.values()),
            'physical_s':2*args.pickup_s+sum(waves)*args.delivery_s,
            'estimated_total_s':T,'stages':steps}

def milp_select(rows):
    # Variables binarias x_i; una y solo una asignación certificada.
    costs=np.array([r['estimated_total_s'] for r in rows],dtype=float)
    res=milp(costs,integrality=np.ones(len(rows)),bounds=Bounds(np.zeros(len(rows)),np.ones(len(rows))),
             constraints=LinearConstraint(np.ones((1,len(rows))),[1.],[1.]),
             options={'time_limit':60})
    if res.x is None or not res.success:raise RuntimeError(f'MILP: {res.message}')
    i=int(np.argmax(res.x))
    return i,{'solver':'scipy.optimize.milp (HiGHS)','status':res.message,'objective_s':float(res.fun),
              'candidates_certified':len(rows),'scope':'Óptimo solamente entre candidatos CERTIFIED'}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario',type=Path,required=True)
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--max-speed',type=float,default=2.0)
    ap.add_argument('--max-accel',type=float,default=2.0)
    ap.add_argument('--pickup-s',type=float,default=23.98)
    ap.add_argument('--delivery-s',type=float,default=15.98)
    ap.add_argument('--candidates',type=int,default=8)
    ap.add_argument('--include-baseline',action='store_true',help='Planifica además la asignación nearest original (más tiempo)')
    ap.add_argument('--iterations',type=int,default=2)
    ap.add_argument('--maxiter',type=int,default=450)
    ap.add_argument('--margin',type=float,default=.05)
    ap.add_argument('--max-ct',type=int,default=20000)
    ap.add_argument('--max-time',type=int,default=65)
    ap.add_argument('--library-dir', type=Path, default=None,
                    help='Guarda automáticamente la mejor solución en la biblioteca por semilla')
    a=ap.parse_args()
    if a.library_dir is not None and (abs(a.max_speed-2.0)>1e-8 or abs(a.max_accel-2.0)>1e-8):
        ap.error('La biblioteca usa solo max-speed=2.0 y max-accel=2.0')
    if not (1<=a.candidates<=720) or min(a.max_speed,a.max_accel,a.pickup_s,a.delivery_s)<=0:ap.error('Argumentos inválidos')
    scenario=load(a.scenario)
    bridge=PlanningBridge(scenario)
    trash=tuple(sorted(bridge.pending))
    if len(trash)!=6:raise ValueError('El escenario debe contener 6 basuras')
    candidates=[]
    for perm in itertools.permutations(trash):
        pair=(perm[:3],perm[3:])
        candidates.append((proxy_cost(scenario,pair),pair))
    candidates.sort(key=lambda x:(x[0],x[1]))
    shortlist=[p for _,p in candidates[:a.candidates]]
    if a.include_baseline:
        try:
            baseline=base_assignment(scenario)
            if baseline not in shortlist:shortlist.insert(0,baseline)
        except Exception as ex:
            print('Aviso: no se pudo obtener baseline:',ex,flush=True)
    a.output_dir.mkdir(parents=True,exist_ok=True)
    certified=[];failures=[]
    for idx,pair in enumerate(shortlist,1):
        print(f'[{idx}/{len(shortlist)}] pickup1={pair[0]} pickup2={pair[1]}',flush=True)
        try:
            rec=evaluate(scenario,pair,a)
            certified.append(rec)
            print(f'  CERTIFIED tiempo={rec["estimated_total_s"]:.2f}s oleadas={rec["waves_delivery"]}',flush=True)
        except Exception as ex:
            failures.append({'pickup1':list(pair[0]),'pickup2':list(pair[1]),'reason':str(ex)})
            print(f'  REJECTED: {ex}',flush=True)
        (a.output_dir/'avance.json').write_text(json.dumps({'certified':certified,'rejected':failures},indent=2,ensure_ascii=False))
    if certified:
        best,solver=milp_select(certified)
        report={'schema':'swarm.d9923.offline_milp_bezier.v1','seed':scenario.seed,
                'params':vars(a)|{'scenario':str(a.scenario),'output_dir':str(a.output_dir)},
                'all_assignments':len(candidates),'shortlisted':len(shortlist),
                'milp':solver,'best':certified[best],'certified':certified,'rejected':failures,
                'limitations':['MILP óptimo solo entre asignaciones candidatas certificadas',
                'Preselección geométrica NO es costo Bézier',
                'Tiempo físico estimado 23.98/15.98 s no es máximo garantizado',
                'Oleadas físicas modeladas como costo aditivo; verificar sincronización y retornos reales',
                'No se ejecuta ROS ni Gazebo ni se garantiza seguridad física']}
        print('MEJOR:',report['best']['estimated_total_s'],'s',report['best']['pickup1'],report['best']['pickup2'])
    else:
        report={'schema':'swarm.d9923.offline_milp_bezier.v1','status':'NO_CERTIFIED_CANDIDATES',
                'shortlisted':len(shortlist),'rejected':failures}
        print('Sin candidatos certificados; ver informe')
    path=a.output_dir/'report.json';path.write_text(json.dumps(report,indent=2,ensure_ascii=False,default=str))
    print('Reporte:',path)
    if certified and a.library_dir is not None:
        import subprocess,sys
        tool=Path(__file__).resolve().with_name('d9924_assignment_library.py')
        cp=subprocess.run([sys.executable,str(tool),'save','--scenario',str(a.scenario),
                           '--report',str(path),'--library-dir',str(a.library_dir)],check=False)
        if cp.returncode:
            print('BIBLIOTECA: fallo al registrar resultado',flush=True)
            return 2
    return 0 if certified else 2
if __name__=='__main__':raise SystemExit(main())

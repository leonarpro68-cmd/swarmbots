#!/usr/bin/env python3
"""Puente conservador de entrega supermarket -> D8 original.

Por defecto: preflight sin movimiento; --execute SOLO un robot.
La rutina D8 hace giro, aproximación, release y retorno; no se replica aquí.
"""
import argparse, hashlib, json, math, subprocess, sys
from pathlib import Path

ROBOTS=('summit0','summit1','summit2')
NAV_SHA='500234f35039471f15f42801d3c76ebc5686cbe4c480fac6eda98c142fdec0e3'
ENDPOINTS={'summit0':(5.,-10.),'summit1':(-2.5,-1.25),'summit2':(-5.,15.)}
ACK=('DETACH CONFIRMADO','REGRESO CONFIRMADO')

def save(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    temp.replace(path)

def prepare(scenario,assign,nav,robot):
    if scenario.get('seed')!=100 or scenario.get('world')!='supermarket':
        raise RuntimeError('Se requiere escenario supermarket, semilla 100')
    if assign.get('seed')!=100 or assign.get('world')!='supermarket' or assign.get('stage')!='pickup1':
        raise RuntimeError('Asignación pickup1 incompatible')
    if nav.get('stage')!='delivery1' or nav.get('status')!='DONE_ALL_DELIVERY1' or nav.get('commands_sent')!=3 or nav.get('problems'):
        raise RuntimeError('No se acreditó navegación delivery1 de los tres robots')
    if nav.get('trajectory_sha256')!=NAV_SHA:
        raise RuntimeError('Hash de delivery1 diferente del plan certificado')
    entries={e['name']:e for e in scenario.get('elements',[])}
    assigned=assign['robots'][robot]
    trash=assigned['trash']
    color=assigned['color']
    deposit='deposit_'+color
    if (trash not in entries or deposit not in entries or
        entries[trash].get('type')!='trash' or entries[deposit].get('type')!='deposit' or
        entries[trash].get('color')!=color or entries[deposit].get('color')!=color):
        raise RuntimeError('Basura, depósito y color no corresponden al escenario')
    goal=ENDPOINTS[robot]
    dist=math.dist(goal,entries[deposit]['xy_m'])
    if not (0.9<=dist<=1.8):
        raise RuntimeError(f'Destino planificado a {dist:.3f} m del depósito; fuera del rango D8')
    return {'trash':trash,'deposit':deposit,'color':color,'planned_goal_xy_m':list(goal),'deposit_xy_m':entries[deposit]['xy_m'],'planned_distance_to_deposit_m':dist}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot',required=True,choices=ROBOTS)
    ap.add_argument('--scenario',required=True,type=Path)
    ap.add_argument('--assignment',required=True,type=Path)
    ap.add_argument('--navigation-report',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--confirm-loaded',action='store_true',help='Operador confirmó en Gazebo que el robot mantiene la carga')
    ap.add_argument('--confirm-supervision',action='store_true',help='Operador supervisa Gazebo y puede pausar simulación')
    args=ap.parse_args()
    if args.output.exists():ap.error('Reporte ya existe: no se repite una maniobra física automáticamente')
    report={'schema':'supermarket.delivery1.physical.original_d8.v1','stage':'delivery1','robot':args.robot,
            'execution_requested':args.execute,'commands_may_have_been_sent':False,'status':'REJECT_STOPPED','problems':[]}
    try:
        scenario=json.loads(args.scenario.read_text(encoding='utf-8'))
        assign=json.loads(args.assignment.read_text(encoding='utf-8'))
        nav=json.loads(args.navigation_report.read_text(encoding='utf-8'))
        report.update(prepare(scenario,assign,nav,args.robot))
        script=Path(__file__).resolve().parents[1]/'prueba_entrega_local_cbs.py'
        if not script.is_file():raise RuntimeError(f'Falta D8 original: {script}')
        cmd=[sys.executable,'-u',str(script),'--scenario',str(args.scenario.resolve()),
             '--robot',args.robot,'--trash',report['trash'],'--deposit',report['deposit'],
             '--max-start-distance','1.8']
        if args.execute and (not args.confirm_loaded or not args.confirm_supervision):
            raise RuntimeError('Para soltar se requiere --confirm-loaded y --confirm-supervision')
        if args.execute:cmd.append('--execute')
        # D8 valida odometría y distancia; por defecto no publica.
        # En modo execute, primero ejecutamos D8 SIN --execute para comprobar su preflight.
        if args.execute:
            dry=subprocess.run(cmd[:-1],capture_output=True,text=True,timeout=30)
            drylog=args.output.with_name(args.output.stem+'_preflight.log')
            drylog.write_text(dry.stdout+dry.stderr,encoding='utf-8')
            report['d8_preflight_log']=str(drylog)
            if dry.returncode!=0 or 'DRY RUN: no se ha publicado ninguna orden.' not in dry.stdout:
                raise RuntimeError(f'Preflight D8 no apto (exit={dry.returncode}); revisar {drylog}')
            report['commands_may_have_been_sent']=True
            save(args.output,report)
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
        logfile=args.output.with_suffix('.log')
        logfile.write_text(result.stdout+result.stderr,encoding='utf-8')
        report.update({'original_exit_code':result.returncode,'original_log':str(logfile)})
        if result.returncode!=0:
            raise RuntimeError(f'Rutina D8 exit={result.returncode}; revisar {logfile}')
        if args.execute:
            if not all(marker in result.stdout for marker in ACK):
                raise RuntimeError('Faltan ACK de detach/regreso; comprobar estado real de Gazebo')
            report['ack_markers']=list(ACK)
            report['status']='PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'
        else:
            if 'DRY RUN: no se ha publicado ninguna orden.' not in result.stdout:
                raise RuntimeError('D8 no devolvió confirmación de prueba sin comandos')
            report['status']='PREFLIGHT_READY_NO_COMMANDS'
    except Exception as exc:
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        report['status']='REJECT_STOPPED'
    save(args.output,report)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status'] in ('PREFLIGHT_READY_NO_COMMANDS','PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION') else 2

if __name__=='__main__':raise SystemExit(main())

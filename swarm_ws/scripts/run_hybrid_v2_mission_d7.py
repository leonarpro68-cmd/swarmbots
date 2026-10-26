#!/usr/bin/env python3
"""D7: un comando, cuatro etapas; reutiliza los scripts D6 y exige supervisión física.

No declara éxito de manipulación basándose solo en estados de navegación.
No reinicia Gazebo ni sustituye las rutinas originales de agarre/entrega.
"""
import argparse
import json
import subprocess
import time
from datetime import datetime, timezone
import sys
from pathlib import Path

ROBOTS=('summit0','summit1','summit2')
STAGES=('pickup1','delivery1','pickup2','delivery2')

def execute(command, *, dry=False):
    print('\n$ '+' '.join(map(str,command)), flush=True)
    if not dry:
        subprocess.run(list(map(str,command)), check=True)

def yes_or_abort(message):
    response=input('\n'+message+' Escribe SI para continuar: ').strip()
    if response!='SI':
        raise RuntimeError('Interrumpido: confirmación física no concedida; revisar Gazebo y estado antes de reanudar')

def stage_targets(report_file, stage):
    data=json.loads(report_file.read_text(encoding='utf-8'))['stages'][stage]
    key='assigned_names' if stage.startswith('pickup') else 'delivered_names'
    names=data[key]
    if len(names)!=3 or len(set(names))!=3 or any(not n.startswith('trash_') for n in names):
        raise RuntimeError('Asignación del reporte inesperada')
    return names

def deposits(scenario):
    data=json.loads(scenario.read_text(encoding='utf-8'))
    return {e['color']:e['name'] for e in data['elements'] if e.get('type')=='deposit'}

def delivery_schedule(names, deposit_lookup):
    # En cada depósito las acciones son serializadas; por simplicidad todas son secuenciales.
    mapping=[]
    for robot,trash in zip(ROBOTS,names):
        color=trash.rsplit('_',1)[-1]
        if color not in deposit_lookup:
            raise RuntimeError('No existe depósito para '+trash)
        mapping.append((robot,trash,deposit_lookup[color]))
    # Los depósitos con un solo robot primero; el depósito compartido después.
    counts={d:sum(x[2]==d for x in mapping) for _,_,d in mapping}
    return sorted(mapping,key=lambda x:(counts[x[2]],x[2],x[0]))

def cli():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--state',type=Path,default=None,help='Estado existente o ruta nueva; por defecto dentro de la carpeta de ejecución')
    p.add_argument('--output-dir',type=Path,default=None,help='Carpeta de ejecución; por defecto crear mision_001, mision_002, ...')
    p.add_argument('--world-verified',action='store_true',help='Operador verificó escenario Gazebo, odom y geometría (incluida carga)')
    p.add_argument('--execute',action='store_true',help='Autoriza la ejecución física con puntos de confirmación')
    p.add_argument('--auto-confirm',action='store_true',help='Verificación automática de cada agarre/entrega por ACK + poses Gazebo; se detiene ante incertidumbre')
    p.add_argument('--skip-start-confirmation', action='store_true', help='Omite el SI inicial; requiere --auto-confirm, --world-verified y --execute')
    p.add_argument('--start-at',choices=STAGES,help='Opcional: reanudar solo si coincide con el ledger')
    p.add_argument('--max-speed',type=float,default=1.2)
    p.add_argument('--max-accel',type=float,default=1.0)
    a=p.parse_args()
    if a.max_speed!=1.2 or a.max_accel!=1.0:
        p.error('D7 conserva 1.20 m/s y 1.00 m/s², como la misión validada')
    if a.skip_start_confirmation and (not a.execute or not a.world_verified or not a.auto_confirm):
        p.error('--skip-start-confirmation requiere --execute --world-verified --auto-confirm')
    if a.execute and not a.world_verified:
        p.error('--execute requiere --world-verified, previa comprobación real en Gazebo')
    if not a.scenario.is_file():p.error('No se encuentra escenario')
    a.scenario=a.scenario.resolve()
    if a.state is not None:a.state=a.state.resolve()
    if a.output_dir is not None:a.output_dir=a.output_dir.resolve()
    return a

def main():
    a=cli(); py=sys.executable;root=Path(__file__).resolve().parent
    def script(name):
        candidate=root/name
        if not candidate.is_file():raise RuntimeError('Falta dependencia '+str(candidate))
        return str(candidate)
    if not a.execute:
        print('DRY RUN DE ORQUESTACIÓN: no se inicia ROS ni se modifican archivos. Para movimiento: --world-verified --execute')
        print('ETAPAS:',', '.join(STAGES))
        for dependency in ('d6_mission_state.py','prepare_hybrid_v2_d6_stage.py','retime_hybrid_v2_d51.py',
                           'convert_d51_protocol_pieces.py','run_hybrid_v2_stage_ros.py',
                           'd6_grasp_existing.py','prueba_entrega_local_cbs.py',
                           'prueba_alinear_agarrar_cbs.py','agarres_paralelos_cbs.py', 'd7_parallel_delivery.py'):
            script(dependency)
        print('DEPENDENCIAS OK; paso siguiente: ejecutar bajo supervisión')
        return
    if a.output_dir is None:
        parent=(Path('scripts/resultados/d7')).resolve()
        parent.mkdir(parents=True,exist_ok=True)
        for n in range(1,100000):
            candidate=parent/f'mision_{n:03d}'
            try:
                candidate.mkdir(exist_ok=False)
                a.output_dir=candidate
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError('No hay identificadores de misión libres')
    else:
        a.output_dir.mkdir(parents=True,exist_ok=True)
    if a.state is None:
        a.state=a.output_dir/'mision.json'
    print(f'D7 RUN DIRECTORY: {a.output_dir}',flush=True)
    print(f'D7 LEDGER: {a.state}',flush=True)
    if not a.state.exists():
        execute([py,script('d6_mission_state.py'),'--state',a.state,'init','--scenario',a.scenario])
    state=json.loads(a.state.read_text())
    completed=state.get('completed',[])
    if completed!=list(STAGES[:len(completed)]) or len(completed)>4:
        raise RuntimeError('Ledger D6 incoherente; no continuar')
    next_stage=STAGES[len(completed)] if len(completed)<4 else None
    if a.start_at and a.start_at!=next_stage:
        raise RuntimeError(f'--start-at={a.start_at} pero la próxima etapa real es {next_stage}')
    if next_stage is None:
        print('MISSION COMPLETE: ya registrado, no repetir');return
    if a.skip_start_confirmation:
        print('[D7] Confirmación SI inicial omitida por opción explícita; se mantienen preflight y verificaciones automáticas.', flush=True)
    else:
        yes_or_abort('Confirma al INICIO: Gazebo y /clock correctos, tres Summit detenidos, odometría mundo verificada, geometría de carga y corredores locales revisados.')
    if a.auto_confirm:
        from d7_auto_evidence import checked_positions, verify_pickup, verify_delivery
        scene=json.loads(a.scenario.read_text(encoding='utf-8'))
        depot_xy={e['name']:tuple(map(float,e['xy_m'])) for e in scene['elements'] if e.get('type')=='deposit'}
        evidence=a.output_dir/'auto_evidence.jsonl'
        def record(result):
            result['at_utc']=datetime.now(timezone.utc).isoformat()
            with evidence.open('a',encoding='utf-8') as f:f.write(json.dumps(result,ensure_ascii=False)+'\n')
            print('AUTOVERIFICADO:',result,flush=True)
    dep=deposits(a.scenario)
    from d7_parallel_delivery import release_waves
    from agarres_paralelos_cbs import parallel_grasps
    from d6_grasp_existing import (
        check_preapproach, snapshot_odom, scenario_targets,
    )
    from d7_parallel_delivery import segment_distance
    import math
    scene_data=json.loads(a.scenario.read_text(encoding='utf-8'))
    trash_info={e['name']:e for e in scene_data['elements'] if e.get('type')=='trash'}
    trash_xy=scenario_targets(scene_data)
    depot_xy_all={e['name']:tuple(map(float,e['xy_m'])) for e in scene_data['elements'] if e.get('type')=='deposit'}
    def parallel_pickup_preflight(names):
        """Global preflight of ALL THREE before the FIRST local command.

        The per-robot D6 guard was designed for stationary neighbors; for
        simultaneous maneuvers we ALSO require disjoint approach corridors.
        These are conservative 2D approximations, not continuous certification.
        """
        from d7_auto_evidence import checked_positions
        odom=snapshot_odom()
        observed=checked_positions()
        for robot,name in zip(ROBOTS,names):
            check_preapproach(robot,name,odom,trash_xy,observed,
                              max_distance=2.2,target_tolerance=.20,separation=1.15)
        for i,ra in enumerate(ROBOTS):
            for rb in ROBOTS[i+1:]:
                # Same conservative separation between trash objectives as the
                # ORIGINAL parallel pickup executor.
                if math.dist(trash_xy[names[ROBOTS.index(ra)]],
                             trash_xy[names[ROBOTS.index(rb)]]) < 3.0:
                    raise RuntimeError(f'AGARRE PARALELO BLOQUEADO: metas de {ra}/{rb} a menos de 3.0 m')
                a0=odom[ra]['xy']; a1=trash_xy[names[ROBOTS.index(ra)]]
                b0=odom[rb]['xy']; b1=trash_xy[names[ROBOTS.index(rb)]]
                gap=segment_distance(a0,a1,b0,b1)
                if gap < 1.15:
                    raise RuntimeError(f'AGARRE PARALELO BLOQUEADO: corredores {ra}/{rb} separados '
                                       f'{gap:.3f} m < 1.15 m. Revisar posiciones; sin movimiento')
        print('PREFLIGHT PARALELO OK: tres objetivos, corredores y odometrías revisados.',flush=True)
        return observed
    for stage in STAGES[len(completed):]:
        print('\n'+'='*22+' '+stage.upper()+' '+'='*22,flush=True)
        base=a.output_dir/stage
        if base.exists() and any(base.iterdir()):
            raise RuntimeError(f'Existen datos de {stage}: {base}. No sobrescribir ejecución parcial: inspeccionar y reanudar con nueva carpeta.')
        base.mkdir(parents=True,exist_ok=True)
        raw_r=base/f'{stage}_live_report.json';raw_c=base/f'{stage}_live_controls.npz'
        ret=base/'retimed';proto=base/'protocol'
        execute([py,script('prepare_hybrid_v2_d6_stage.py'),'--scenario',a.scenario,'--state',a.state,
                 '--stage',stage,'--accept-namespaced-world-odom','--radius','0.55','--margin','0.05',
                 '--max-speed','1.20','--max-accel','1.00','--iterations','2','--output-dir',base])
        execute([py,script('retime_hybrid_v2_d51.py'),'--report',raw_r,'--controls',raw_c,
                 '--stage',stage,'--max-speed','1.20','--max-accel','1.00','--safety-factor','1.08',
                 '--subdivisions','4','--output-dir',ret])
        execute([py,script('convert_d51_protocol_pieces.py'),'--report',ret/f'{stage}_retimed_report.json',
                 '--controls',ret/f'{stage}_retimed_controls.npz','--stage',stage,'--output-dir',proto])
        report=proto/f'{stage}_protocol_report.json';controls=proto/f'{stage}_protocol_controls.npz'
        # Reintentos del proceso de preflight por discovery; nunca sobre un --execute.
        for attempt in range(1,4):
            try:
                execute([py,script('run_hybrid_v2_stage_ros.py'),'--report',report,'--controls',controls,
                         '--stage',stage,'--scenario',a.scenario]);break
            except subprocess.CalledProcessError:
                if attempt==3:raise
                print(f'PRECHECK no aprobado (intento {attempt}/3). Comprobar grafo ROS y nueva odometría.',flush=True)
                if not a.auto_confirm:
                    yes_or_abort('¿Has comprobado que los 3 path_follower siguen activos y no hubo movimiento?')
                time.sleep(1.0)
        if not a.auto_confirm:
            if stage.startswith('delivery'):
                yes_or_abort('Confirma tres objetos sujetos, huella total dentro del radio 0.55 m y zona de navegación segura.')
            else:
                yes_or_abort('Confirma que los tres robots están libres de carga y la escena coincide con el escenario.')
        else:
            # Comprobación conservadora: antes de navegar con carga, cada basura debe acompañar al robot.
            if stage.startswith('delivery'):
                names_check=stage_targets(raw_r,stage)
                poses=checked_positions()
                from d6_grasp_existing import snapshot_odom
                odom=snapshot_odom()
                for rob,trash in zip(ROBOTS,names_check):
                    if trash not in poses or __import__('math').dist(poses[trash],odom[rob]['xy'])>1.05:
                        raise RuntimeError(f'{rob}: la carga {trash} no está cerca; no navegar')
            else:
                checked_positions()
        execute([py,script('run_hybrid_v2_stage_ros.py'),'--report',report,'--controls',controls,
                 '--stage',stage,'--scenario',a.scenario,'--robot-radius','0.55',
                 '--operational-margin','0.05','--world-verified','--execute'])
        names=stage_targets(raw_r,stage)
        if stage.startswith('pickup'):
            before=parallel_pickup_preflight(names)
            assignments=dict(zip(ROBOTS,names))
            receipts=[]
            print('AGARRE PARALELO: iniciando las 3 rutinas originales simultáneamente',flush=True)
            parallel_grasps(
                scenario=a.scenario, assignments=assignments, trash=trash_info,
                script=Path(script('prueba_alinear_agarrar_cbs.py')),
                logfile=base/'parallel_pickups.log',
                on_success=lambda robot,trash:receipts.append((robot,trash)),
                separation_m=3.0, timeout_s=110.0,
            )
            if set(receipts)!=set(assignments.items()):
                raise RuntimeError('BARRERA PICKUP: faltan ACKs de uno o más Summit')
            if a.auto_confirm:
                # The old parallel executor calls on_success as robots finish.
                # Verify only AFTER all have returned; snapshot_odom expects
                # ALL robots stationary.
                for robot,trash in assignments.items():
                    record(verify_pickup(robot,trash,before))
            else:
                yes_or_abort('Confirma en Gazebo que las tres basuras están sujetas y los tres Summit regresaron.')
        else:
            assignments=dict(zip(ROBOTS,names))
            # Recheck the carried objects AFTER global navigation, not only before.
            if a.auto_confirm:
                positions=checked_positions()
                odom=snapshot_odom()
                for robot,trash in assignments.items():
                    if trash not in positions or math.dist(positions[trash],odom[robot]['xy'])>1.05:
                        raise RuntimeError(f'ENTREGA BLOQUEADA: carga {trash} no acompaña a {robot}')
            receipts=[]
            def on_release(robot,trash):
                receipts.append((robot,trash))
            def after_wave(wave):
                # Barrier: all members returned BEFORE starting the next wave.
                if a.auto_confirm:
                    for robot in wave:
                        trash=assignments[robot]
                        color=trash.rsplit('_',1)[-1]
                        record(verify_delivery(robot,trash,dep[color],depot_xy_all))
                else:
                    yes_or_abort(f'Comprueba entregas de oleada {wave}, detach real y regreso de todos.')
            def before_wave(wave, starts, ends):
                # Verify that stationary waiting robots and returning robots
                # have NOT moved off the positions used by the wave scheduler.
                current=snapshot_odom()
                for robot in ROBOTS:
                    offset=math.dist(current[robot]['xy'],starts[robot])
                    if offset>0.12:
                        raise RuntimeError(f'OLEADA BLOQUEADA: {robot} cambió {offset:.3f}m '
                                           'respecto a la geometría validada; replantear oleadas')
                if a.auto_confirm:
                    observed=checked_positions()
                    for robot in wave:
                        trash=assignments[robot]
                        if trash not in observed or math.dist(observed[trash],current[robot]['xy'])>1.05:
                            raise RuntimeError(f'OLEADA BLOQUEADA: carga {trash} de {robot} no corroborada')
            print('ENTREGAS EN OLEADAS: como máximo un Summit por depósito a la vez',flush=True)
            release_waves(
                scenario=a.scenario, assignments=assignments,
                trash=trash_info, deposits=dep,
                transport={'status':'arrived_not_delivered','carried':assignments,
                           'reference_xy':{r:list(d['xy']) for r,d in snapshot_odom().items()}},
                script=Path(script('prueba_entrega_local_cbs.py')),
                logfile=base/'parallel_deliveries.log',
                state_callback=on_release, on_wave_complete=after_wave, on_wave_start=before_wave,
                max_start_distance=1.80, clearance_m=1.25, timeout_s=100.0,
            )
            if set(receipts)!=set(assignments.items()):
                raise RuntimeError('BARRERA DELIVERY: faltan ACKs de uno o más Summit')
        execute([py,script('d6_mission_state.py'),'--state',a.state,'confirm','--stage',stage,
                 '--report',report,'--controls',controls,'--confirm-physical'])
        print('ETAPA VALIDADA:',stage,flush=True)
    print('\nMISSION COMPLETE: seis basuras entregadas en cuatro etapas; revisar registros y Gazebo.',flush=True)

if __name__=='__main__':
    try: main()
    except (RuntimeError,subprocess.CalledProcessError,KeyboardInterrupt) as ex:
        print('\nD7 STOP: '+str(ex)+'\nNO reanudar a ciegas. Inspeccionar estado físico de Gazebo y ledger.',file=sys.stderr)
        sys.exit(2)

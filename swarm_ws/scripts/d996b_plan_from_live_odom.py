#!/usr/bin/env python3
"""D9.9.6B: prepare pickup1 with fresh Gazebo odometry, entirely BEFORE motion.

Calls ONLY these pre-existing planning / offline validation scripts:
  prepare_hybrid_v2_d6_stage.py (ROS subscriptions only)
  retime_hybrid_v2_d51.py
  convert_d51_protocol_pieces.py
  d99_collective_phase_retime.py
  d994_convert_collective.py
  d995_preflight_collective.py
No ROS cmd_vel publishers, no navigation action, no Gazebo spawning.

Output is explicitly NOT EXECUTABLE and must not be given to a follower yet.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone
import numpy as np

STAGE = 'pickup1'
REQUIRED = (
    'd6_mission_state.py', 'prepare_hybrid_v2_d6_stage.py',
    'retime_hybrid_v2_d51.py', 'convert_d51_protocol_pieces.py',
    'd99_collective_phase_retime.py', 'd994_convert_collective.py',
    'd995_preflight_collective.py', 'd9_offline_retime.py',
    'd94_convert_offline.py', 'd92_smooth_retime.py',
    'd91_segment_retime.py',
)


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def check_inputs(scripts, scenario, world, mode, source_run):
    needed=REQUIRED if mode=='live' else ('d99_collective_phase_retime.py', 'd994_convert_collective.py', 'd995_preflight_collective.py', 'd9_offline_retime.py', 'd94_convert_offline.py', 'd92_smooth_retime.py', 'd91_segment_retime.py')
    missing=[name for name in needed if not (scripts/name).is_file()]
    if missing: raise ValueError('Scripts requeridos no encontrados: '+', '.join(missing))
    if mode == 'live':
        if not scenario or not scenario.is_file() or not world or not world.is_file():
            raise ValueError('Live requiere --scenario y --world-sdf existentes')
        scene=json.loads(scenario.read_text(encoding='utf8'))
        el=scene.get('elements',[])
        if not isinstance(el,list) or sum(e.get('type')=='trash' for e in el)!=6 or sum(e.get('type')=='deposit' for e in el)!=3:
            raise ValueError('Escenario no contiene exactamente 6 basuras y 3 depósitos')
        if source_run: raise ValueError('No se usa --source-run con --mode live')
    elif mode == 'existing':
        if source_run is None:raise ValueError('--source-run requerido en modo existing')
        if not (source_run/'mission'/STAGE/'protocol'/f'{STAGE}_protocol_controls.npz').is_file():
            raise ValueError('Fuente D8 existing no contiene el protocolo de pickup1')
    else:raise ValueError('Modo desconocido')


def compare_live_start(run, tolerance_m=1e-6):
    """Ensure the D8 report's freshly sampled odom is EXACT source geometry start."""
    stage_dir=run/'mission'/STAGE
    raw=json.loads((stage_dir/f'{STAGE}_live_report.json').read_text(encoding='utf8'))
    h=raw['stages'][STAGE]['hybrid_v2']
    if not (raw['stages'][STAGE].get('certified') and h.get('certified') and h.get('status')=='CERTIFIED'):
        raise ValueError('Plan Hybrid V2 de odometría fresca no certificado')
    if h.get('odom_frame_policy') not in ('verified_namespaced_world_coordinates','shared_frame'):
        raise ValueError('Política de marcos sin verificación')
    odom=h.get('odom_xy')
    if not isinstance(odom,(list,tuple)) or len(odom)!=3:
        raise ValueError('No hay 3 poses de odometría en el reporte D8')
    with np.load(stage_dir/'protocol'/f'{STAGE}_protocol_controls.npz',allow_pickle=False) as npz:
        starts=[np.asarray(npz[f'R{i}'][0,0],dtype=float) for i in range(3)]
    errors=[float(np.linalg.norm(starts[i]-np.asarray(odom[i],dtype=float))) for i in range(3)]
    if any(not math.isfinite(e) or e>tolerance_m for e in errors):
        raise ValueError(f'El protocolo D8 no inicia exactamente en la odometría: {errors}')
    return {'odom_initial_xy_m':odom,'protocol_initial_xy_m':[x.tolist() for x in starts],
            'source_odom_to_protocol_start_errors_m':errors,
            'odom_frame_policy':h['odom_frame_policy'],
            'odom_frames':h.get('odom_frames'),
            'odom_snapshot_time_s':h.get('odom_snapshot_time_s')}


def run_cmd(scripts, name, flags, log):
    argv=[sys.executable,str(scripts/name),*map(str,flags)]
    with log.open('a',encoding='utf8') as f:
        f.write('\n$ '+ ' '.join(argv)+'\n');f.flush()
        result=subprocess.run(argv,stdout=f,stderr=subprocess.STDOUT,check=False)
        f.write(f'EXIT_CODE={result.returncode}\n');f.flush()
    if result.returncode:
        raise RuntimeError(f'{name}: exit={result.returncode}, revisar {log}')


def make_stage(scripts, run, scenario, world, mode):
    stage_dir=run/'mission'/STAGE
    stage_dir.mkdir(parents=True,exist_ok=False)
    log=run/'pipeline.log'
    if mode=='live':
        state=run/'mission_state_for_planning_only.json'
        run_cmd(scripts,'d6_mission_state.py', ['--state',state,'init','--scenario',scenario],log)
        run_cmd(scripts,'prepare_hybrid_v2_d6_stage.py',
                ['--scenario',scenario,'--state',state,'--stage',STAGE,
                 '--accept-namespaced-world-odom','--radius','0.55',
                 '--margin','0.05','--max-speed','1.20','--max-accel','1.00',
                 '--iterations','2','--output-dir',stage_dir],log)
        raw_r=stage_dir/f'{STAGE}_live_report.json'
        raw_c=stage_dir/f'{STAGE}_live_controls.npz'
        ret=stage_dir/'retimed';proto=stage_dir/'protocol'
        run_cmd(scripts,'retime_hybrid_v2_d51.py',
                ['--report',raw_r,'--controls',raw_c,'--stage',STAGE,
                 '--max-speed','1.20','--max-accel','1.00',
                 '--safety-factor','1.08','--subdivisions','4','--output-dir',ret],log)
        run_cmd(scripts,'convert_d51_protocol_pieces.py',
                ['--report',ret/f'{STAGE}_retimed_report.json',
                 '--controls',ret/f'{STAGE}_retimed_controls.npz',
                 '--stage',STAGE,'--output-dir',proto],log)
    return log


def offline_pipeline(scripts, run, output):
    log=output/'pipeline.log'
    d99=output/'d99';converted=output/'converted';preflight=output/'preflight'
    run_cmd(scripts,'d99_collective_phase_retime.py',
            ['--run',run,'--stage',STAGE,'--out',d99],log)
    r=json.loads((d99/f'{STAGE}_d99_report.json').read_text())
    if r['status']!='CERTIFIED_OFFLINE_COLLECTIVE':
        raise ValueError('El reescalado colectivo rechazó las trayectorias originales')
    run_cmd(scripts,'d994_convert_collective.py',
            ['--run',run,'--stage',STAGE,'--out',converted],log)
    run_cmd(scripts,'d995_preflight_collective.py',
            ['--run',run,'--stage',STAGE,'--conversion',converted,'--out',preflight],log)
    cv=json.loads((converted/f'{STAGE}_d994_conversion_report.json').read_text())
    pf=json.loads((preflight/f'{STAGE}_d995_preflight.json').read_text())
    if cv['status']!='CONVERTED_OFFLINE_CHECKS_PASSED' or pf['status']!='PASS_OFFLINE_READ_ONLY':
        raise ValueError('La conversión o el preflight D9.9 rechazaron la solución')
    with np.load(converted/f'{STAGE}_d994_NON_EXECUTABLE.npz',allow_pickle=False) as z:
        pos=[np.asarray(z[f'R{i}_controls'][0,0],dtype=float) for i in range(3)]
    with np.load(run/'mission'/STAGE/'protocol'/f'{STAGE}_protocol_controls.npz',allow_pickle=False) as z:
        orig=[np.asarray(z[f'R{i}'][0,0],dtype=float) for i in range(3)]
    geom_errors=[float(np.linalg.norm(a-b)) for a,b in zip(pos,orig)]
    if max(geom_errors)>1e-7:raise ValueError('La conversión modificó el punto de partida')
    return r,cv,pf,[x.tolist() for x in pos],geom_errors


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['live','existing'],default='live')
    ap.add_argument('--scenario',type=Path,help='Escenario EXACTO cargado en Gazebo')
    ap.add_argument('--world-sdf',type=Path,help='World SDF EXACTO de Gazebo; identidad del mundo cargado sigue pendiente')
    ap.add_argument('--source-run',type=Path,help='Solo para regresión offline de un run D8 anterior')
    ap.add_argument('--output-dir',type=Path,required=True,help='Carpeta NUEVA, nunca se sobrescribe')
    a=ap.parse_args()
    scripts=Path(__file__).resolve().parent
    scenario=a.scenario.resolve() if a.scenario else None
    world=a.world_sdf.resolve() if a.world_sdf else None
    source_run=a.source_run.resolve() if a.source_run else None
    output=a.output_dir.resolve()
    check_inputs(scripts,scenario,world,a.mode,source_run)
    if output.exists():ap.error('La carpeta --output-dir ya existe; usar una carpeta NUEVA')
    output.mkdir(parents=True,exist_ok=False)
    manifest_path=output/'d996b_manifest.json'
    manifest={'schema':'swarm.d996b.fresh_start.offline.v1','status':'INCOMPLETE',
              'mode':a.mode,'stage':STAGE,'execution_authorized':False,
              'ros_commands_sent':0,'navigation_started':False,
              'created_utc':datetime.now(timezone.utc).isoformat(),
              'scenario':str(scenario) if scenario else None,
              'world_sdf':str(world) if world else None,
              'source_run':str(source_run) if source_run else None,
              'warnings':['No publica comandos de movimiento',
                          'NO certifica que Gazebo cargó exactamente los bytes de world_sdf',
                          'NO certifica TF ni comportamiento del seguidor en movimiento']}
    try:
        run=output if a.mode=='live' else source_run
        if a.mode=='live':
            make_stage(scripts,run,scenario,world,a.mode)
            start=compare_live_start(run)
            manifest['fresh_start']=start
        r,cv,pf,starts,errors=offline_pipeline(scripts,run,output)
        manifest['converted_start_xy_m']={f'summit{i}':xy for i,xy in enumerate(starts)}
        manifest['conversion_to_protocol_start_errors_m']=errors
        if a.mode=='live':
            refs=[np.asarray(s,dtype=float) for s in manifest['fresh_start']['odom_initial_xy_m']]
            exact=[float(np.linalg.norm(x-y)) for x,y in zip(refs,starts)]
            if max(exact)>1e-6:raise ValueError('El inicio D9.9.4 ya no coincide con el snapshot de odometría')
            manifest['snapshot_to_converted_start_errors_m']=exact
        manifest['timing']={'D51_original_s':r['original_makespan_s'],
                            'D99_collective_s':r['optimized_makespan_s'],
                            'finishing_times_s':r['finishing_times_s'],
                            'phase_scales':r['phase_scales_after_events']}
        manifest['artifacts']={
            'source_protocol':str(run/'mission'/STAGE/'protocol'/f'{STAGE}_protocol_controls.npz'),
            'collective_report':str(output/'d99'/f'{STAGE}_d99_report.json'),
            'converted_trajectory':str(output/'converted'/f'{STAGE}_d994_NON_EXECUTABLE.npz'),
            'conversion_report':str(output/'converted'/f'{STAGE}_d994_conversion_report.json'),
            'offline_preflight':str(output/'preflight'/f'{STAGE}_d995_preflight.json'),
            'world_sdf':str(world) if world else None,
            'scenario':str(scenario) if scenario else None}
        manifest['sha256']={k:sha256(v) for k,v in manifest['artifacts'].items() if v}
        manifest['offline_preflight_status']=pf['status']
        manifest['status']='PASS_OFFLINE_FRESH_START_NOT_EXECUTABLE' if a.mode=='live' else 'PASS_OFFLINE_REGRESSION_NOT_EXECUTABLE'
    except (Exception,) as exc:
        manifest['status']='REJECT_PIPELINE';manifest['error']=str(exc)
        manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
        print(json.dumps({'status':manifest['status'],'error':manifest['error'], 'manifest':str(manifest_path)},indent=2))
        return 2
    manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print(json.dumps({'status':manifest['status'],'manifest':str(manifest_path),
                      'start_xy':manifest['converted_start_xy_m'],
                      'timing':manifest['timing']},indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())

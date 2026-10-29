#!/usr/bin/env python3
"""D9.9.19: supervisor integrado de navegación, agarre y descarga por oleadas.

Mantiene las barreras de seguridad de D9.9.17 y D9.9.18. No fabrica
certificados de despeje ni confirma entrega solo por recibir ACK.
La misión queda PAUSED_CLEARANCE_REQUIRED cuando D9.8C no tiene ejecución
física de despeje verificada. No ejecuta automáticamente wave2.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
ROBOTS = ('summit0', 'summit1', 'summit2')
SHAS = {
    'pickup1':'3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd',
    'delivery1':'70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b',
    'pickup2':'7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11',
    'delivery2':'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60',
}


def write(path, data):
    tmp=path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n', encoding='utf8')
    os.replace(tmp,path)


def step(root, report, label, args, result_path, expected):
    command=[sys.executable, *map(str,args)]
    print('[D9919]',label,flush=True)
    with (root/'pipeline.log').open('a',encoding='utf8') as log:
        log.write('\n$ '+' '.join(command)+'\n');log.flush()
        output=subprocess.run(command,capture_output=True,text=True,errors='replace')
        log.write(output.stdout+'\n'+output.stderr+'\nEXIT='+str(output.returncode)+'\n')
    if output.stdout: print(output.stdout[-1200:],flush=True)
    try: data=json.loads(result_path.read_text(encoding='utf8'))
    except (ValueError,OSError) as exc: raise RuntimeError(f'{label}: sin informe válido; consultar pipeline.log: {exc}')
    status=data.get('status')
    report['steps'].append({'step':label,'status':status,'exit_code':output.returncode,'file':str(result_path)})
    write(root/'report.json',report)
    if output.returncode!=0 or status not in expected:
        raise RuntimeError(f'{label}: estado {status!r}, exit {output.returncode}; revisar {result_path}')
    return data


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',help='Permite navegación y agarres/descargas reales; no wave2 sin despeje')
    ap.add_argument('--confirm-runtime-v2',action='store_true')
    ap.add_argument('--confirm-cargo-inspected',action='store_true',help='Confirmación humana de joint real, antes de la ejecución')
    ap.add_argument('--output-dir',default='scripts/resultados/d9919_mision_001')
    ap.add_argument('--scripts-dir',default='scripts')
    ap.add_argument('--reference-dir',default='scripts/d996_reference')
    ap.add_argument('--ledger',default='scripts/resultados/d8/seed_100_run_007/mission/mision.json')
    ap.add_argument('--scenario',default='scripts/d996_reference/scenario.json')
    a=ap.parse_args()
    root=Path(a.output_dir).resolve()
    if root.exists(): ap.error('No se sobrescriben corridas anteriores; elige otra carpeta')
    root.mkdir(parents=True)
    scripts=Path(a.scripts_dir).resolve()
    ref=Path(a.reference_dir).resolve()
    report={'schema':'swarm.d9919.physical_mission.v1','status':'INITIALIZING',
            'created_utc':datetime.now(timezone.utc).isoformat(),
            'execute':a.execute,'steps':[],'completed_stages':[],
            'physical_deliveries_verified':False,'problems':[]}
    write(root/'report.json',report)
    try:
        if a.execute and not a.confirm_runtime_v2:
            raise RuntimeError('Falta --confirm-runtime-v2')
        files={'align':scripts/'d997_initial_alignment_path.py',
               'nav':scripts/'d9913_stage_coordinator.py',
               'physical':scripts/'d9917_physical_stage.py',
               'waves':scripts/'d9918_delivery_waves.py'}
        for name,p in files.items():
            if not p.is_file(): raise RuntimeError(f'No existe módulo {name}: {p}')
        # Verificar identidad del escenario, ledger y trayectorias antes de mover.
        scenario=Path(a.scenario).resolve()
        ledger=Path(a.ledger).resolve()
        from pathlib import Path as _P
        scenario_hash=hashlib.sha256(scenario.read_bytes()).hexdigest()
        metadata=json.loads(ledger.read_text())
        if metadata.get('scenario_sha256')!=scenario_hash:
            raise RuntimeError('Escenario distinto al ledger histórico')
        refs={}
        for stage in STAGES:
            p=ref/(stage+'_d994_NON_EXECUTABLE.npz')
            if hashlib.sha256(p.read_bytes()).hexdigest()!=SHAS[stage]:
                raise RuntimeError(f'{stage}: NPZ no coincide con referencia histórica')
            refs[stage]=p
        report['status']='PREFLIGHT_REFERENCES_VALID'
        write(root/'report.json',report)
        if not a.execute:
            path=root/'pickup1_alignment_dry.json'
            step(root,report,'pickup1_alignment_dry',
                 [files['align'],'--trajectory',refs['pickup1'],'--output',path],path,
                 {'DRY_RUN_READY','ALREADY_ALIGNED'})
            report['status']='DRY_RUN_REFERENCES_AND_PICKUP1_OK_NO_COMMANDS'
            write(root/'report.json',report)
            return 0

        pickup_dir=root/'agarres'
        pickup_dir.mkdir()
        for stage in STAGES:
            src=refs[stage]
            align_path=root/(stage+'_alignment.json')
            step(root,report,stage+'_alignment',
                 [files['align'],'--trajectory',src,'--execute','--output',align_path],align_path,
                 {'ALIGNED','ALREADY_ALIGNED'})
            nav_dry=root/(stage+'_navigation_dry.json')
            step(root,report,stage+'_navigation_dry',
                 [files['nav'],'--stage',stage,'--output',nav_dry],nav_dry,
                 {'PASS_GUARDED_DRY_RUN_NO_COMMANDS'})
            nav_out=root/(stage+'_navigation.json')
            step(root,report,stage+'_navigation',
                 [files['nav'],'--stage',stage,'--execute','--confirm-runtime-v2','--output',nav_out],nav_out,
                 {'DONE_ALL_'+stage.upper()})
            if stage.startswith('pickup'):
                # Serializar agarres D8 hasta certificar que son seguros en paralelo.
                for robot in ROBOTS:
                    p=pickup_dir/(stage+'_'+robot+'_execute.json')
                    step(root,report,stage+'_'+robot+'_physical_pickup',
                         [files['physical'],'--stage',stage,'--robot',robot,
                          '--navigation-report',nav_out,'--execute','--output',p],p,
                         {'PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'})
            else:
                plan=root/(stage+'_wave_plan.json')
                step(root,report,stage+'_wave_plan',
                     [files['waves'],'--stage',stage,'--navigation-report',nav_out,
                      '--pickup-evidence-dir',pickup_dir,'--out',plan],plan,
                     {'WAVE_PLAN_READY_NO_COMMANDS'})
                if not a.confirm_cargo_inspected:
                    report['status']='PAUSED_CARGO_INSPECTION_REQUIRED'
                    report['pending_stage']=stage
                    report['pending_wave']=1
                    report['problems'].append('No se ha acreditado el joint de cada carga en Gazebo; ACK no basta')
                    write(root/'report.json',report)
                    print('[D9919] PARADA SEGURA: falta inspección física de carga')
                    return 3
                wave1=root/(stage+'_wave1.json')
                step(root,report,stage+'_wave1',
                     [files['waves'],'--stage',stage,'--navigation-report',nav_out,
                      '--pickup-evidence-dir',pickup_dir,'--execute-wave','1',
                      '--confirm-cargo-inspected','--out',wave1],wave1,
                     {'WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION'})
                # D9.8C solo calcula rutas CBS offline, NO ejecuta en Gazebo ni
                # confirma la liberación del acceso. Nunca abrir wave2 sin ello.
                report['status']='PAUSED_CLEARANCE_EXECUTOR_NOT_VALIDATED'
                report['pending_stage']=stage
                report['pending_wave']=2
                report['completed_wave1_report']=str(wave1)
                report['problems'].append('Falta ejecutar y comprobar despeje CBS D9.8C en Gazebo antes de wave2; detener aquí')
                write(root/'report.json',report)
                print('[D9919] PARADA SEGURA: oleada 1 ejecutada. No se permite wave2 sin despeje real CBS + odometría')
                return 3
            report['completed_stages'].append(stage)
            write(root/'report.json',report)
        report['status']='DONE_ALL_FOUR_PHYSICAL_STAGES'
        write(root/'report.json',report)
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        report['status']='REJECT_STOPPED'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        write(root/'report.json',report)
        print('[D9919] DETENIDO:',exc,file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())

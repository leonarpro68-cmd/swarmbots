#!/usr/bin/env python3
"""Muestra comandos D9.9.7/D9.9.13; NO ejecuta ninguno."""
import argparse,hashlib,json
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--prepared',type=Path,required=True)
 p.add_argument('--stage',choices=('pickup1','delivery1','pickup2','delivery2'),required=True)
 p.add_argument('--run-id',default='milp_seed100_01')
 a=p.parse_args()
 m=json.loads((a.prepared/'phase5_static_preflight.json').read_text())
 if m['status']!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE':p.error('Preflight offline no aprobado')
 n=m['stages'][a.stage];traj=Path(n['trajectory']);clock=Path(n['successive_clock'])
 if not traj.is_file() or hashlib.sha256(traj.read_bytes()).hexdigest()!=n['sha256']:
  p.error('Trayectoria MILP cambió o falta')
 if not clock.is_file() or json.loads(clock.read_text())['input_sha256']!=n['sha256']:
  p.error('Reloj V3 cambió')
 out=f'scripts/resultados/{a.run_id}_{a.stage}_REAL.json'
 print('=== SOLO IMPRIME — NO EJECUTA ===')
 print('1) Alinear D9.9.7 (comprobar PRIMERO sin --execute):')
 print(f'python3 scripts/d997_initial_alignment_path.py --trajectory "{traj}" --output "/tmp/{a.run_id}_{a.stage}_align_dry.json"')
 print('2) Dry-run del coordinador D9.9.13:')
 head=f'python3 scripts/d9913_stage_coordinator.py --stage {a.stage} --trajectory "{traj}" --successive-clock "{clock}" --expected-sha256 {n["sha256"]} --nav-speed-limit 2.0 --nav-accel-limit 2.0 --observe-seconds 8 --start-delay 15'
 print(head+f' --output "/tmp/{a.run_id}_{a.stage}_coordinator_dry.json"')
 print('3) Comando REAL solo después de completar pruebas ROS, alineación y autorización del operador:')
 print(head+f' --execute --confirm-runtime-v2 --output "{out}"')
 print('\nNO ejecuta comandos. La fase5 offline nunca autoriza movimiento.')

if __name__=='__main__':main()

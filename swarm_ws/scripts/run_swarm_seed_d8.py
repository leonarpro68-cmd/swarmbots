#!/usr/bin/env python3
"""D8: genera escenario, arranca Gazebo y ejecuta D7 con una semilla.

No mata procesos ROS/Gazebo preexistentes: rechaza una simulación ya activa.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROBOTS = tuple(f'summit{i}' for i in range(3))

def checked(cmd, *, env=None, timeout=300):
    print('+', ' '.join(map(str,cmd)), flush=True)
    subprocess.run([str(x) for x in cmd], check=True, env=env, timeout=timeout)

def probe(cmd, timeout=7):
    try:
        p = subprocess.run(list(map(str,cmd)), capture_output=True, text=True, timeout=timeout)
        return p.returncode == 0, p.stdout + p.stderr
    except (subprocess.TimeoutExpired, OSError):
        return False, ''

def wait_ready(world, model_names, launch, timeout):
    deadline = time.monotonic() + timeout
    last = 'Gazebo/ROS todavía no listos'
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            raise RuntimeError(f'Gazebo/launch terminó prematuramente (código {launch.returncode})')
        clock_ok, clock = probe(['ros2','topic','echo','--once','/clock','rosgraph_msgs/msg/Clock'], timeout=3)
        if not clock_ok:
            last = 'No hay mensajes /clock en ROS 2. Comprueba ros_gz_bridge'; time.sleep(1); continue
        missing = []
        for robot in ROBOTS:
            # ros2 topic echo --once confirma que la odometría se está publicando, no sólo anunciada.
            ok, _ = probe(['ros2','topic','echo','--once',f'/{robot}/odom','nav_msgs/msg/Odometry'], timeout=3)
            if not ok: missing.append(f'/{robot}/odom')
        if missing:
            last = 'Odometrías sin mensajes: '+', '.join(missing); time.sleep(1); continue
        nodes_ok, nodes = probe(['ros2','node','list'], timeout=4)
        wanted = {f'/{r}/path_follower' for r in ROBOTS} | {f'/{r}/visual_grasp' for r in ROBOTS} | {f'/grasp_manager_{r}' for r in ROBOTS}
        found = set(nodes.splitlines()) if nodes_ok else set()
        if not wanted.issubset(found):
            last = 'Nodos faltantes: '+str(sorted(wanted-found)); time.sleep(1); continue
        gz_ok, gz_text = probe(['gz','model','--list'], timeout=8)
        if not gz_ok or not all(name in gz_text for name in model_names):
            last = 'Gazebo no muestra aún los 3 Summit y los 9 objetos del escenario'; time.sleep(1); continue
        print('[D8] READY: /clock, 3 odometrías, 9 nodos de control/manipulación, modelos Gazebo', flush=True)
        return
    raise TimeoutError(f'No se completó el arranque ({timeout:.0f}s): {last}')

def stop_group(proc, grace=12):
    if proc is None or proc.poll() is not None: return
    try: os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError: return
    try: proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        try: os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError: pass
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try: os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError: pass

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output-root', type=Path, default=Path('scripts/resultados/d8'))
    parser.add_argument('--ready-timeout', type=int, default=240)
    parser.add_argument('--world', type=Path, default=None, help='SDF original; por defecto share/swarm_worlds/worlds/tugbot_warehouse.sdf')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--auto-confirm', action='store_true', help='Confirmaciones físicas automáticas D7')
    parser.add_argument('--require-start-confirmation', action='store_true', help='Restaurar solicitud manual SI antes de la misión')
    closure = parser.add_mutually_exclusive_group()
    closure.add_argument('--keep-gazebo', dest='keep_gazebo', action='store_true', default=True, help='Mantener Gazebo abierto al terminar (por defecto); Ctrl+C para cerrar')
    closure.add_argument('--close-gazebo', dest='keep_gazebo', action='store_false', help='Cerrar Gazebo y ROS al terminar la misión')
    parser.add_argument('--execute', action='store_true', help='Genera mundo, arranca Gazebo y ejecuta la misión')
    a = parser.parse_args()
    if a.ready_timeout < 20: parser.error('--ready-timeout debe ser >= 20')
    if a.execute and not a.auto_confirm and not a.require_start_confirmation:
        parser.error('Para omitir SI, usa --auto-confirm; de lo contrario usa --require-start-confirmation')
    scripts = Path(__file__).resolve().parent
    workspace = scripts.parent
    for dep in ['escenario_aleatorio_swarm.py','swarm_cbs_grasp_prepare.py','run_hybrid_v2_mission_d7.py']:
        if not (scripts/dep).is_file(): parser.error(f'Falta {scripts/dep}')
    if a.world is None:
        ok, prefix = probe(['ros2','pkg','prefix','swarm_worlds'])
        if not ok: parser.error('No está disponible swarm_worlds; source install/setup.bash')
        a.world = Path(prefix.strip())/'share/swarm_worlds/worlds/tugbot_warehouse.sdf'
    a.world = a.world.expanduser().resolve()
    if not a.world.is_file(): parser.error(f'No existe mundo base: {a.world}')
    a.output_root = a.output_root.expanduser().resolve()
    if not a.execute:
        print('[D8 DRY RUN] Semilla', a.seed, '| mundo base:', a.world)
        print('[D8 DRY RUN] Usar --execute para generar el escenario y lanzar Gazebo + D7')
        return 0
    # No mezclar la misión con un mundo ya levantado.
    ok, out = probe(['gz','model','--list'], timeout=4)
    if ok and 'Available models:' in out:
        raise RuntimeError('Ya hay un mundo Gazebo accesible. Cierra la simulación previa antes de iniciar D8.')
    a.output_root.mkdir(parents=True, exist_ok=True)
    for n in range(1, 100000):
        run = a.output_root/f'seed_{a.seed}_run_{n:03d}'
        try: run.mkdir(exist_ok=False); break
        except FileExistsError: continue
    else: raise RuntimeError('Se agotaron las carpetas')
    scenario = run/'scenario.json'
    world_sdf = run/'world.sdf'
    checked([sys.executable, scripts/'escenario_aleatorio_swarm.py','--seed',a.seed,'--output',scenario], timeout=600)
    data = json.loads(scenario.read_text())
    if data.get('seed') != a.seed: raise RuntimeError('Semilla JSON incongruente')
    checked([sys.executable,scripts/'swarm_cbs_grasp_prepare.py','--scenario',scenario,'--world',a.world,'--output',world_sdf])
    models = [e['name'] for e in data['elements']] + list(ROBOTS)
    launch_cmd = ['ros2','launch','swarm_worlds','swarm_cbs_grasp_base.launch.py',f'seed:={a.seed}',f'scenario:={scenario}',f'world_sdf:={world_sdf}','hybrid_enabled:=true','hybrid_max_speed:=1.20']
    if a.headless: launch_cmd.append('headless:=true')
    mission_cmd = [sys.executable, scripts/'run_hybrid_v2_mission_d7.py','--scenario',scenario, '--output-dir',run/'mission', '--state',run/'mission'/'mision.json', '--world-verified', '--execute']
    if a.auto_confirm: mission_cmd.append('--auto-confirm')
    if a.auto_confirm and not a.require_start_confirmation: mission_cmd.append('--skip-start-confirmation')
    (run/'commands.json').write_text(json.dumps({'launch':list(map(str,launch_cmd)),'mission':list(map(str,mission_cmd)),'seed':a.seed},indent=2))
    print('[D8] RUN:',run,flush=True)
    launch = None
    bridge = None
    log = (run/'gazebo_launch.log').open('w')
    try:
        print('+',' '.join(map(str, launch_cmd)),flush=True)
        launch=subprocess.Popen(launch_cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=workspace)
        # A veces Gazebo ya provee /clock via el launch existente; no crear segundo puente si ya hay mensajes.
        initial_deadline=time.monotonic()+25
        clock_ok=False
        while time.monotonic()<initial_deadline:
            if launch.poll() is not None: raise RuntimeError('Gazebo cerró durante arranque; revisa gazebo_launch.log')
            clock_ok,_=probe(['ros2','topic','echo','--once','/clock','rosgraph_msgs/msg/Clock'],timeout=3)
            if clock_ok: break
            time.sleep(1)
        if not clock_ok:
            print('[D8] /clock sin bridge; arrancando puente dedicado',flush=True)
            bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=workspace)
        wait_ready('world_demo',models,launch,a.ready_timeout)
        # READY comprueba sensores y nodos; NO comprueba que Gazebo haya usado
        # las posiciones de inicio del escenario CBS. D8.7 exige coincidencia.
        checked([sys.executable, scripts/'d8_verify_initial_positions.py',
                 '--scenario', scenario, '--output', run/'spawn_verification.json',
                 '--max-error', '0.25', '--timeout', '22'], timeout=35)
        if a.require_start_confirmation or not a.auto_confirm:
            print('[D8] Mundo y odometría disponibles. Verifica visualmente antes de autorizar SI.',flush=True)
        else:
            print('[D8] Sin SI inicial: ejecución automática solicitada. READY no garantiza ausencia de colisiones en maniobras locales.',flush=True)
        checked(mission_cmd,timeout=7200)
        print('[D8] MISSION COMPLETE: salida en',run,flush=True)
        if a.keep_gazebo:
            print('[D8] Gazebo permanece abierto. Pulsa Ctrl+C en esta terminal para cerrar los procesos de esta ejecución.',flush=True)
            try:
                while launch.poll() is None:
                    time.sleep(0.5)
                print(f'[D8] Gazebo se cerró (código={launch.returncode}).',flush=True)
            except KeyboardInterrupt:
                print('\n[D8] Cerrando Gazebo por solicitud del operador...', flush=True)
            return 0
        return 0
    finally:
        print('[D8] Cerrando procesos de esta ejecución...',flush=True)
        stop_group(bridge)
        stop_group(launch)
        log.close()

if __name__=='__main__':
    try: sys.exit(main())
    except KeyboardInterrupt: print('\n[D8] Cancelación solicitada',file=sys.stderr); sys.exit(130)
    except Exception as exc: print('[D8] ERROR:',exc,file=sys.stderr); sys.exit(1)

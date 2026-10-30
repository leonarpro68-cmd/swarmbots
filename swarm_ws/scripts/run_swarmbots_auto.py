#!/usr/bin/env python3
"""Un terminal: preparar escenario, iniciar Gazebo + /clock y ejecutar misión automática.

Ejecutar dentro del contenedor ROS 2 con el workspace ya compilado y sourceado.
NO recupera ni reinicia una misión parcial. --dry-run no arranca procesos.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent


def spawn(cmd, logpath):
    f = logpath.open('w', encoding='utf-8')
    p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=f, stderr=subprocess.STDOUT,
                         start_new_session=True, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    f.close()
    return p


def shutdown(procs):
    for p in reversed(procs):
        if p.poll() is None:
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
    for p in reversed(procs):
        try: p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try: os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError: pass
            try: p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                p.wait()


def probe_topic(topic, timeout=4):
    try:
        x = subprocess.run(['ros2', 'topic', 'echo', topic, '--once'],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=timeout, text=True)
        return x.returncode == 0 and bool(x.stdout.strip())
    except (subprocess.TimeoutExpired, OSError):
        return False


def wait_ready(gz, bridge, wait):
    end = time.monotonic() + wait
    odoms = ('/summit0/odom', '/summit1/odom', '/summit2/odom')
    print('Esperando /clock y odometrías (sin mover robots)...', flush=True)
    while time.monotonic() < end:
        if gz.poll() is not None or bridge.poll() is not None:
            raise RuntimeError('Gazebo o bridge terminó antes de estar preparado')
        if probe_topic('/clock', 3) and all(probe_topic(t, 3) for t in odoms):
            print('Preparación ROS básica confirmada.', flush=True)
            return
        time.sleep(1)
    raise RuntimeError('Tiempo agotado esperando /clock y las tres odometrías')


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--seed', type=int, default=123)
    a.add_argument('--world-original', type=Path, required=True,
                   help='SDF base usado previamente para generar los mundos probados')
    a.add_argument('--output-dir', type=Path, default=Path('/tmp/swarm_autonomo_v11'))
    a.add_argument('--ready-timeout', type=int, default=150)
    a.add_argument('--dry-run', action='store_true')
    a.add_argument('--close-on-success', action='store_true',
                   help='Restaurar el cierre automático de Gazebo tras una misión exitosa')
    args = a.parse_args()
    if not args.world_original.is_file():
        a.error('No existe --world-original')
    for f in ('escenario_aleatorio_swarm.py', 'swarm_cbs_grasp_prepare.py',
              'mision_fisica_completa_auto.py', 'mision_fisica_ronda1_auto.py',
              'mision_fisica_ronda2_auto.py', 'entregas_paralelas_cbs_auto.py'):
        if not (HERE/f).is_file():
            a.error(f'Falta {HERE/f}. Copia todos los scripts del paquete')
    d = args.output_dir / f'seed_{args.seed}_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}'
    scenario = d/'scenario.json'; world = d/'world.sdf'; prefix = d/'mission'
    print(f'SwARMbOTS AUTO | seed={args.seed} | registro={d}', flush=True)
    if args.dry_run:
        print('DRY RUN: no se generan escenarios ni se arrancan procesos.', flush=True)
        return 0
    d.mkdir(parents=True, exist_ok=False)
    procs = []
    try:
        subprocess.run([sys.executable, str(HERE/'escenario_aleatorio_swarm.py'),
                        '--seed', str(args.seed), '--output', str(scenario)], check=True)
        subprocess.run([sys.executable, str(HERE/'swarm_cbs_grasp_prepare.py'),
                        '--scenario', str(scenario), '--world', str(args.world_original),
                        '--output', str(world)], check=True)
        s = world.read_text(encoding='utf-8')
        s = re.sub(r'(<\?xml\s+[^>]*encoding\s*=\s*[\'\"])ASCII([\'\"])',
                   r'\1UTF-8\2', s, count=1, flags=re.I)
        world.write_text(s, encoding='utf-8')
        ET.parse(world)
        print('Iniciando Gazebo...', flush=True)
        gz = spawn(['ros2', 'launch', 'swarm_worlds', 'swarm_cbs_grasp_base.launch.py',
                    f'seed:={args.seed}', f'scenario:={scenario}', f'world_sdf:={world}'],
                   d/'gazebo.log')
        procs.append(gz)
        bridge = spawn(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], d/'clock_bridge.log')
        procs.append(bridge)
        wait_ready(gz, bridge, args.ready_timeout)
        print('Ejecutando las dos rondas sin preguntas por teclado...', flush=True)
        mission = [sys.executable, '-u', str(HERE/'mision_fisica_completa_auto.py'),
                   '--scenario', str(scenario), '--prefix', str(prefix), '--execute']
        with (d/'mission_console.log').open('w', encoding='utf-8') as log:
            m = subprocess.Popen(mission, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1,
                                 env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            try:
                for line in m.stdout:
                    print(line, end='', flush=True)
                    log.write(line); log.flush()
                code = m.wait()
            except KeyboardInterrupt:
                m.terminate()
                try: m.wait(timeout=8)
                except subprocess.TimeoutExpired: m.kill(); m.wait()
                raise
        print(f'Registros conservados en {d}', flush=True)
        if code:
            print('Misión detenida; no se relanza ni se reanuda sobre este mundo.', file=sys.stderr, flush=True)
            print('La misión FALLÓ: Gazebo queda abierto solamente para inspección.', file=sys.stderr, flush=True)
        else:
            print('ACK completos; la colocación dentro del depósito no se ha verificado automáticamente.', flush=True)
        # --close-on-success afecta solo a los éxitos; ante fallo dejamos el
        # mundo intacto para inspección, sin iniciar más movimiento ni reintentos.
        if code or not args.close_on_success:
            print('Gazebo permanece abierto para inspección. Presiona Ctrl+C para cerrarlo.', flush=True)
            try:
                while gz.poll() is None and bridge.poll() is None:
                    time.sleep(0.5)
                print('Gazebo o el bridge terminó; cerrando los procesos restantes.', flush=True)
            except KeyboardInterrupt:
                print('Cerrando Gazebo y el bridge tras la inspección...', flush=True)
        return code
    except KeyboardInterrupt:
        print('Interrupción: detener procesos; NO reanudar sobre Gazebo parcialmente ejecutado.', file=sys.stderr)
        return 130
    except (RuntimeError, OSError, subprocess.CalledProcessError, ET.ParseError) as e:
        print(f'ERROR: {e}; revisar {d}', file=sys.stderr)
        return 1
    finally:
        shutdown(procs)

if __name__ == '__main__':
    sys.exit(main())

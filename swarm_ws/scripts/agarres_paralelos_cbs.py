#!/usr/bin/env python3
"""Ejecución concurrente SUPERVISADA de tres agarres ya asignados por CBS.

No publica cmd_vel ni modifica CBS. Ejecuta un proceso por namespace y
solo retorna con tres ACK (ALINEAR, JOINT, REGRESO). Ante fallo, detener
la misión y revisar Gazebo antes de cualquier reintento.
"""
from __future__ import annotations

import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')
MARKERS = ('ALINEAR CONFIRMADO', 'JOINT CONFIRMADO', 'REGRESO CONFIRMADO')


def _stop(processes):
    for p in processes.values():
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    for p in processes.values():
        if p.poll() is None:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()


def _pipe_lines(robot, pipe, q):
    try:
        for line in pipe:
            q.put((robot, line))
    finally:
        pipe.close()


def parallel_grasps(*, scenario: Path, assignments: dict, trash: dict,
                    script: Path, logfile: Path, on_success,
                    separation_m: float = 3.0, timeout_s: float = 110.0):
    """on_success(robot, trash_name) persiste el ACK de cada agarre.

    Garantía de concurrencia: tres procesos ROS diferentes, uno por robot.
    No se pretende garantizar evitación local de obstáculos: las zonas se
    comprueban por separación de centros, más inspección visual humana.
    """
    if set(assignments) != set(ROBOTS) or len(set(assignments.values())) != 3:
        raise RuntimeError('Asignaciones inválidas o repetidas')
    xy = {}
    for robot, name in assignments.items():
        if name not in trash:
            raise RuntimeError(f'Objeto desconocido: {name}')
        xy[robot] = tuple(map(float, trash[name]['xy_m']))
    for i, ra in enumerate(ROBOTS):
        for rb in ROBOTS[i+1:]:
            gap = math.dist(xy[ra], xy[rb])
            if gap < separation_m:
                raise RuntimeError(
                    f'PRECHECK SEGURIDAD: {ra} y {rb} tienen objetos a {gap:.2f} m '
                    f'(< {separation_m:.2f} m). Ejecutar agarres secuenciales.')
    print('\nAGARRES PARALELOS: objetivos separados; una maniobra por Summit.', flush=True)
    print('PRECAUCIÓN: la separación entre objetos NO certifica trayectorias libres.', flush=True)
    processes = {}
    threads = []
    q = queue.Queue()
    outputs = {r: [] for r in ROBOTS}
    recorded = set()
    logfile.parent.mkdir(parents=True, exist_ok=True)
    try:
        with logfile.open('a', encoding='utf-8') as log:
            log.write('\n\n### AGARRES PARALELOS\n')
            for robot in ROBOTS:
                cmd = [sys.executable, str(script), '--scenario', str(scenario),
                       '--robot', robot, '--trash', assignments[robot]]
                log.write(f'{robot}: {" ".join(cmd)}\n')
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1, start_new_session=True,
                                        env={**os.environ, 'PYTHONUNBUFFERED': '1'})
                processes[robot] = proc
                t = threading.Thread(target=_pipe_lines, args=(robot, proc.stdout, q), daemon=True)
                t.start()
                threads.append(t)
            deadline = time.monotonic() + timeout_s
            while True:
                try:
                    robot, line = q.get(timeout=.2)
                    outputs[robot].append(line)
                    print(f'[{robot}] {line}', end='', flush=True)
                    log.write(f'[{robot}] {line}')
                    log.flush()
                except queue.Empty:
                    pass
                for robot, proc in processes.items():
                    code = proc.poll()
                    if code is not None and robot not in recorded:
                        # Esperar el lector para no perder últimas líneas del stdout.
                        threads[list(processes).index(robot)].join(timeout=1)
                        while True:
                            try:
                                name, line = q.get_nowait()
                            except queue.Empty:
                                break
                            outputs[name].append(line)
                            print(f'[{name}] {line}', end='', flush=True)
                            log.write(f'[{name}] {line}')
                        result = ''.join(outputs[robot])
                        missing = [m for m in MARKERS if m not in result]
                        if code or missing:
                            raise RuntimeError(f'AGARRE {robot} falló: exit={code}, '
                                               f'ACK faltantes={missing}; revisar posición y joints')
                        recorded.add(robot)
                        on_success(robot, assignments[robot])
                        print(f'AGARRE CONFIRMADO [{robot}]: {assignments[robot]}', flush=True)
                if len(recorded) == len(ROBOTS):
                    print('BARRERA AGARRE COMPLETA: 3 JOINT + 3 REGRESOS CONFIRMADOS', flush=True)
                    return
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Timeout agarres paralelos ({timeout_s}s); '
                                       f'confirmados={sorted(recorded)}')
    finally:
        _stop(processes)
        for t in threads:
            t.join(timeout=1)

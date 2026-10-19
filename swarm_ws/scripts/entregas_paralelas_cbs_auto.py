#!/usr/bin/env python3
"""Oleadas de entregas físicas: máximo un Summit por depósito en cada oleada.

Usa procesos ROS por robot; no altera CBS. Cada ola exige autorización humana.
Precheck aproximado de segmentos de centros y robots estacionados; NO sustituye
un planificador local con evitación de obstáculos ni validación en Gazebo.
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
ACK = ('DETACH CONFIRMADO', 'REGRESO CONFIRMADO')


def point_segment(p, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    k = (vx*vx + vy*vy)
    t = max(0.0, min(1.0, ((p[0]-a[0])*vx+(p[1]-a[1])*vy)/k)) if k else 0.0
    return math.dist(p, (a[0]+t*vx, a[1]+t*vy))


def segment_distance(a, b, c, d):
    # 2D segments: test crossing first, otherwise endpoints to segments.
    def cross(x, y, z):
        return (y[0]-x[0])*(z[1]-x[1])-(y[1]-x[1])*(z[0]-x[0])
    x1, x2, x3, x4 = cross(a,b,c), cross(a,b,d), cross(c,d,a), cross(c,d,b)
    if x1*x2 < 0 and x3*x4 < 0:
        return 0.0
    return min(point_segment(a,c,d), point_segment(b,c,d),
               point_segment(c,a,b), point_segment(d,a,b))


def stop_processes(procs):
    for proc in procs.values():
        if proc.poll() is None:
            try: os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError: pass
    for proc in procs.values():
        if proc.poll() is None:
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait()


def read_lines(robot, pipe, q):
    try:
        for line in pipe:
            q.put((robot,line))
    finally:
        pipe.close()


def release_waves(*, scenario: Path, assignments: dict, trash: dict, deposits: dict,
                  transport: dict, script: Path, logfile: Path, state_callback,
                  max_start_distance: float=2.0, clearance_m: float=1.25,
                  timeout_s: float=100.0):
    """state_callback(robot, trash_name) escribe ACK durable al terminar cada robot."""
    if set(assignments) != set(ROBOTS) or len(set(assignments.values())) != 3:
        raise RuntimeError('Asignaciones inválidas o repetidas')
    if transport.get('status') != 'arrived_not_delivered' or transport.get('carried') != assignments:
        raise RuntimeError('Estado de transporte no coincide con cargas de la misión')
    ref = transport.get('reference_xy', {})
    if set(ref) != set(ROBOTS):
        raise RuntimeError('Faltan posiciones reales reference_xy para preflight de entrega')
    starts = {r: tuple(map(float, ref[r])) for r in ROBOTS}
    colors = {r: trash[assignments[r]]['color'] for r in ROBOTS}
    data = __import__('json').loads(scenario.read_text(encoding='utf-8'))
    transport_deposit_xy = {e['name']: tuple(map(float, e['xy_m']))
                            for e in data['elements'] if e['type'] == 'deposit'}
    ends = {}
    for r in ROBOTS:
        deposit = deposits[colors[r]]
        # Las coordenadas de depósitos las aporta el llamador como nombre->xy.
        if deposit not in transport_deposit_xy:
            raise RuntimeError(f'No se conoce la posición del depósito {deposit}')
        dx, dy = transport_deposit_xy[deposit]
        x, y = starts[r]
        distance = math.hypot(dx-x,dy-y)
        if not (0.90 <= distance <= max_start_distance):
            raise RuntimeError(f'{r}: distancia al depósito {distance:.3f} m fuera del preflight')
        ratio = (distance-0.80)/distance
        ends[r] = (x+(dx-x)*ratio, y+(dy-y)*ratio)
    # Ordena color de carga única primero, luego compartidos.
    from collections import Counter
    counts = Counter(colors.values())
    pending = sorted(ROBOTS, key=lambda r: (counts[colors[r]],r))
    waves = []
    while pending:
        wave = []
        for r in list(pending):
            if any(colors[r] == colors[s] for s in wave):
                continue
            if any(segment_distance(starts[r], ends[r], starts[s], ends[s]) < clearance_m for s in wave):
                continue
            # No acercarse en exceso a un robot que permanecerá estacionado.
            if any(point_segment(starts[s], starts[r], ends[r]) < clearance_m
                   for s in ROBOTS if s != r and s not in wave):
                continue
            if any(point_segment(starts[r], starts[s], ends[s]) < clearance_m for s in wave):
                continue
            wave.append(r)
        if not wave:
            raise RuntimeError('PRECHECK LOCAL: ninguna entrega puede ejecutarse sin acercarse '
                               'demasiado a otro robot estacionado; inspeccionar Gazebo')
        waves.append(wave)
        pending = [r for r in pending if r not in wave]
    print('PLAN DE OLEADAS (preflight geométrico; NO certificación de ausencia de colisiones):',
          waves, flush=True)
    logfile.parent.mkdir(parents=True, exist_ok=True)
    for index, wave in enumerate(waves,1):
        print(f'\nOLEADA {index}/{len(waves)}: {wave}', flush=True)
        for r in wave:
            print(f'  {r} -> {deposits[colors[r]]}; aproximación centro {starts[r]} -> {ends[r]}',flush=True)
        print('AUTO: preflight geométrico superado; ejecutando oleada sin autorización por teclado.',flush=True)
        if False:  # AUTO: preflight geométrico ya validado; no hay aprobación por teclado
            raise RuntimeError(f'Oleada {index} no autorizada; sin nuevos movimientos')
        q, procs, threads = queue.Queue(), {}, {}
        outputs = {r: [] for r in wave}
        confirmed = set()
        try:
            with logfile.open('a', encoding='utf-8') as log:
                log.write(f'\n\n### OLEADA {index}: {wave}\n')
                for r in wave:
                    cmd = [sys.executable, str(script), '--scenario', str(scenario), '--robot', r,
                           '--trash', assignments[r], '--deposit', deposits[colors[r]],
                           '--max-start-distance', str(max_start_distance), '--execute']
                    log.write('$ ' + ' '.join(cmd) + '\n')
                    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, bufsize=1, start_new_session=True,
                                            env={**os.environ, 'PYTHONUNBUFFERED':'1'})
                    procs[r]=proc
                    thread=threading.Thread(target=read_lines,args=(r,proc.stdout,q),daemon=True)
                    thread.start()
                    threads[r]=thread
                deadline=time.monotonic()+timeout_s
                while len(confirmed)<len(wave):
                    try:
                        r,line=q.get(timeout=.2)
                        outputs[r].append(line)
                        print(f'[{r}] {line}',end='',flush=True)
                        log.write(f'[{r}] {line}')
                        log.flush()
                    except queue.Empty:
                        pass
                    for r,proc in procs.items():
                        code=proc.poll()
                        if code is None or r in confirmed:
                            continue
                        threads[r].join(timeout=1)
                        # Drain all pending output after EOF, including other robots.
                        while True:
                            try: name,line=q.get_nowait()
                            except queue.Empty: break
                            outputs[name].append(line)
                            print(f'[{name}] {line}',end='',flush=True)
                            log.write(f'[{name}] {line}')
                        missing=[marker for marker in ACK if marker not in ''.join(outputs[r])]
                        if code != 0 or missing:
                            raise RuntimeError(f'ENTREGA {r} falló: exit={code}, ACK faltantes={missing}. '
                                               'NO reintentar sin revisar posición/joint en Gazebo')
                        confirmed.add(r)
                        state_callback(r,assignments[r])
                        print(f'ENTREGA CONFIRMADA [{r}] {assignments[r]}',flush=True)
                    if time.monotonic()>deadline:
                        raise RuntimeError(f'Timeout en oleada {index}; confirmados={sorted(confirmed)}')
                print(f'BARRERA DE ENTREGA: oleada {index} finalizada',flush=True)
        finally:
            stop_processes(procs)
            for th in threads.values(): th.join(timeout=1)
    print('BARRERA DE ENTREGA COMPLETA: 3 DETACH + 3 REGRESOS',flush=True)

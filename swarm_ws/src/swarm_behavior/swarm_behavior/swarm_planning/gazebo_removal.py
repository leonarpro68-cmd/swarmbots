"""Servicios Gazebo para recogida confirmada; sin dependencias ROS 2.

Los timeouts son ambiguos: si no hay confirmación explícita no se libera el mapa.
"""
from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class RemovalResult:
    confirmed: tuple[str, ...]
    failures: dict[str, str]
    service: str


def find_remove_service(world=None, *, run=subprocess.run):
    if world:
        if not re.fullmatch(r'[A-Za-z0-9_]+', world):
            raise ValueError('Nombre de mundo Gazebo inválido')
        return f'/world/{world}/remove'
    proc = run(['gz', 'service', '-l'], capture_output=True, text=True, timeout=15)
    if proc.returncode:
        raise RuntimeError((proc.stderr or proc.stdout).strip() or 'No se pudieron listar servicios')
    services = re.findall(r'^\s*(/world/[^\s/]+/remove)\s*$', proc.stdout, re.MULTILINE)
    if len(services) != 1:
        raise RuntimeError(f'Se esperaba un servicio de eliminación; encontrados: {services}. Usa --world')
    return services[0]


def remove_model(name, service, *, run=subprocess.run):
    if not re.fullmatch(r'[A-Za-z0-9_]+', name):
        raise ValueError('Nombre de basura inválido')
    cmd = ['gz', 'service', '-s', service, '--reqtype', 'gz.msgs.Entity',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '15000',
           '--req', f'name: "{name}" type: MODEL']
    proc = run(cmd, capture_output=True, text=True, timeout=25)
    response = (proc.stdout or '') + '\n' + (proc.stderr or '')
    if proc.returncode != 0 or not re.search(r'\bdata\s*:\s*true\b', response, re.I):
        raise RuntimeError('Gazebo no confirmó la eliminación: ' + response.strip())


def remove_batch(names, world=None, *, find_service=find_remove_service, remove=remove_model):
    names = tuple(names)
    if not names or len(set(names)) != len(names):
        raise ValueError('La lista de recogidas debe contener nombres únicos')
    service = find_service(world)
    confirmed, failures = [], {}
    for name in names:
        try:
            remove(name, service)
        except Exception as exc:
            failures[name] = str(exc)
        else:
            confirmed.append(name)
    return RemovalResult(tuple(confirmed), failures, service)

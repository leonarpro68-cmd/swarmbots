#!/usr/bin/env python3
"""Read-only ROS readiness preflight shared by all prepared SwarmBots missions.

Does not authorize a mission by itself and does not bypass geometric/D8 checks.
Extracted from the retired seed-100 runner without changing its readiness gates.
"""
import pathlib
import subprocess
import sys
import time

def check_live(ws, scenario, timeout):
    deadline=time.monotonic()+timeout
    observer=ws/'scripts/supermarket_preflight/ros_observer_readonly.py'
    support=ws/'scripts/supermarket_support/check_support_graph.py'
    if not observer.is_file() or not support.is_file():
        raise FileNotFoundError('Falta preflight ROS de sólo lectura')
    last=''
    while time.monotonic()<deadline:
        try:
            p=subprocess.run([sys.executable,str(support),'--n-robots','3'],capture_output=True,text=True,timeout=25,cwd=ws)
            if p.returncode==0 and 'PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION' in p.stdout:
                q=subprocess.run([sys.executable,str(observer),'--scenario',str(scenario),'--seconds','8',
                     '--output',str(ws/'scripts/resultados/ros_preflight_readonly.json')],capture_output=True,text=True,timeout=45,cwd=ws)
                if q.returncode==0 and 'PASS_READ_ONLY_NOT_EXECUTION_AUTHORIZATION' in q.stdout:
                    print('ROS_PREFLIGHT_PASS: graph + initial world/odom positions',flush=True)
                    return
                last=f'observer: code={q.returncode}, output={q.stdout[-450:]} {q.stderr[-250:]}'
            else:last=f'graph: code={p.returncode}, output={p.stdout[-300:]} {p.stderr[-200:]}'
        except (subprocess.TimeoutExpired, OSError) as ex:last=str(ex)
        time.sleep(3)
    raise RuntimeError('ROS no está listo o la escena NO coincide con la semilla inicial: '+last)



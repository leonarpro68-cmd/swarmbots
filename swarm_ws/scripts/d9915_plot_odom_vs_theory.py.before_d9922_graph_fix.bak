#!/usr/bin/env python3
"""D9.9.15: gráfica comparativa teoría vs odometría por etapa.

Genera una figura estilo D8/D9.9 con 4 filas x 2 columnas:
izquierda rapidez, derecha aceleración absoluta.
En cada subplot superpone teoría (línea sólida) y realidad por odometría
(línea discontinua) para summit0..2.
"""
import argparse
import json
from math import comb
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
TITLES = {
    'pickup1': 'Recogida 1',
    'delivery1': 'Entrega 1',
    'pickup2': 'Recogida 2',
    'delivery2': 'Entrega 2',
}
COLORS = {'summit0': 'tab:blue', 'summit1': 'tab:green', 'summit2': 'tab:red'}
ROBOT_INDEX = {'summit0': 0, 'summit1': 1, 'summit2': 2}


def moving_average(y, win=7):
    y = np.asarray(y, dtype=float)
    if len(y) == 0 or win <= 1 or len(y) < win:
        return y
    kernel = np.ones(win, dtype=float) / float(win)
    ypad = np.pad(y, (win // 2, win - 1 - win // 2), mode='edge')
    return np.convolve(ypad, kernel, mode='valid')


def sample_theory(npz_path: Path, robot_index: int, target_dt: float = 0.05):
    with np.load(npz_path) as d:
        ctrls = np.asarray(d[f'R{robot_index}_controls'], dtype=float)
        durs = np.asarray(d[f'R{robot_index}_durations_s'], dtype=float)
    times = []
    speeds = []
    accs = []
    t0 = 0.0
    for P, T in zip(ctrls, durs):
        n = max(8, int(np.ceil(float(T) / target_dt)))
        us = np.linspace(0.0, 1.0, n, endpoint=False)
        V = 5.0 * np.diff(P, axis=0) / float(T)
        A = 20.0 * np.diff(P, n=2, axis=0) / float(T) ** 2
        for u in us:
            v = sum(comb(4, k) * (1 - u) ** (4 - k) * u ** k * V[k] for k in range(5))
            a = sum(comb(3, k) * (1 - u) ** (3 - k) * u ** k * A[k] for k in range(4))
            times.append(t0 + u * float(T))
            speeds.append(float(np.linalg.norm(v)))
            accs.append(float(np.linalg.norm(a)))
        t0 += float(T)
    # Include exact endpoint
    u = 1.0
    P = ctrls[-1]
    T = float(durs[-1])
    V = 5.0 * np.diff(P, axis=0) / T
    A = 20.0 * np.diff(P, n=2, axis=0) / T ** 2
    v = sum(comb(4, k) * (1 - u) ** (4 - k) * u ** k * V[k] for k in range(5))
    a = sum(comb(3, k) * (1 - u) ** (3 - k) * u ** k * A[k] for k in range(4))
    times.append(t0)
    speeds.append(float(np.linalg.norm(v)))
    accs.append(float(np.linalg.norm(a)))
    return np.asarray(times), np.asarray(speeds), np.asarray(accs)


def load_odom(jsonl_path: Path):
    data = {r: [] for r in ROBOT_INDEX}
    with jsonl_path.open('r', encoding='utf-8') as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if 'robot' not in obj:
                continue
            robot = obj['robot']
            if robot not in data:
                continue
            data[robot].append(obj)
    for robot in data:
        data[robot].sort(key=lambda x: x['clock_s'])
    return data


def extract_real(records, start_s, end_s):
    times = []
    speeds = []
    vxs = []
    vys = []
    for rec in records:
        c = float(rec['clock_s'])
        if c < start_s or c > end_s:
            continue
        times.append(c - start_s)
        speeds.append(float(rec.get('speed_m_s', 0.0)))
        vxs.append(float(rec.get('vx_m_s', 0.0)))
        vys.append(float(rec.get('vy_m_s', 0.0)))
    if len(times) < 2:
        return np.asarray(times), np.asarray(speeds), np.asarray([])
    times = np.asarray(times)
    speeds = np.asarray(speeds)
    vxs = np.asarray(vxs)
    vys = np.asarray(vys)
    dt = np.diff(times)
    ax = np.divide(np.diff(vxs), dt, out=np.zeros_like(dt), where=np.abs(dt) > 1e-9)
    ay = np.divide(np.diff(vys), dt, out=np.zeros_like(dt), where=np.abs(dt) > 1e-9)
    acc = np.sqrt(ax * ax + ay * ay)
    acc_t = (times[1:] + times[:-1]) * 0.5
    return times, moving_average(speeds, 5), (acc_t, moving_average(acc, 9))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--odom-jsonl', required=True)
    ap.add_argument('--stage-windows', required=True,
                    help='JSON con ventanas por etapa: {stages:[{stage,trajectory,start_time_s,end_time_s}]})')
    ap.add_argument('--output', default='scripts/resultados/d9915_velocidad_aceleracion_real_vs_teorica.png')
    ap.add_argument('--mission-label', default='seed_100_run_007')
    args = ap.parse_args()

    odom = load_odom(Path(args.odom_jsonl).resolve())
    windows = json.loads(Path(args.stage_windows).read_text(encoding='utf-8'))
    stages = {x['stage']: x for x in windows['stages']}

    fig, axes = plt.subplots(4, 2, figsize=(16, 18))
    fig.patch.set_facecolor('#f2f2f2')
    for ax_row in axes:
        for ax in ax_row:
            ax.set_facecolor('#f2f2f2')
            ax.grid(True, alpha=0.25)

    for i, stage in enumerate(STAGES):
        row = axes[i]
        entry = stages[stage]
        traj = Path(entry['trajectory'])
        start_s = float(entry['start_time_s'])
        end_s = float(entry['end_time_s'])

        ax_speed, ax_acc = row
        ax_speed.set_title(f"{TITLES[stage]} · Rapidez (m/s)", fontsize=14, fontweight='bold')
        ax_acc.set_title(f"{TITLES[stage]} · Aceleración |a| (m/s²)", fontsize=14, fontweight='bold')

        for robot, idx in ROBOT_INDEX.items():
            color = COLORS[robot]
            tt, sv, sa = sample_theory(traj, idx)
            rt, rs, ra = extract_real(odom[robot], start_s, end_s)
            ax_speed.plot(tt, sv, color=color, lw=2.4, label=f'{robot.replace("summit", "Summit ")} teórica')
            if len(rt):
                ax_speed.plot(rt, rs, color=color, lw=1.6, ls='--', alpha=0.9,
                              label=f'{robot.replace("summit", "Summit ")} real')
            ax_acc.plot(tt, sa, color=color, lw=2.4, label=f'{robot.replace("summit", "Summit ")} teórica')
            if len(ra) == 2 and len(ra[0]):
                ax_acc.plot(ra[0], ra[1], color=color, lw=1.6, ls='--', alpha=0.9,
                            label=f'{robot.replace("summit", "Summit ")} real')

        ax_speed.axhline(1.2, color='0.35', ls='--', lw=1.4, label='Límite 1.2')
        ax_acc.axhline(1.0, color='0.35', ls='--', lw=1.4, label='Límite 1')
        ax_speed.set_xlabel('Tiempo de trayectoria (s)')
        ax_speed.set_ylabel('Rapidez (m/s)')
        ax_acc.set_xlabel('Tiempo de trayectoria (s)')
        ax_acc.set_ylabel('Aceleración |a| (m/s²)')
        # Legend only in first row to mimic compact style.
        if i == 0:
            ax_speed.legend(loc='upper right', ncol=4, fontsize=9)
            ax_acc.legend(loc='upper right', ncol=4, fontsize=9)

    fig.suptitle(f'SwarmBots · comparación teórico vs real por odometría · D9.9 · {args.mission_label}',
                 fontsize=20, fontweight='bold', y=0.995)
    fig.text(0.5, 0.012,
             'Líneas sólidas: planificación offline D9.9. Líneas discontinuas: odometría en Gazebo. '
             'La aceleración real se estima derivando la velocidad de odometría y se suaviza para visualización.',
             ha='center', fontsize=10)
    out = Path(args.output).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0.02, 0.03, 0.98, 0.985))
    fig.savefig(out, dpi=180)
    print(json.dumps({'status': 'PLOT_READY', 'output': str(out)}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

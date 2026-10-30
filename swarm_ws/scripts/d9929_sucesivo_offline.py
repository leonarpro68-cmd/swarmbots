#!/usr/bin/env python3
"""D9.9.29 — Escalado temporal colectivo sucesivo, prototipo OFFLINE.

El reloj virtual tau es COMUN para todos los robots: x_r(t)=B_r(tau(t)).
q(tau)=d(tau)/dt cambia suavemente tras el fin de cada robot.
No modifica las geometrías ni emite NPZ ejecutables o comandos ROS.

Cotas conservadoras (por segmento Bézier de grado 5):
  |v| <= V_cp*q
  |a| <= A_cp*q^2 + V_cp*q*|dq/dtau|
El incremento de q es quintico smoothstep con derivada nula en los extremos.
La correspondencia simultánea x_i(t),x_j(t) conserva exactamente todas las
configuraciones relativas existentes de la planificación CBS de entrada.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
from numpy.polynomial.legendre import leggauss

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
ROBOTS = ('summit0', 'summit1', 'summit2')
_GAUSS_X, _GAUSS_W = leggauss(32)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_json(path, obj):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + '.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
    os.replace(tmp, p)


def load_stage(path):
    """Admite NPZ D9.9.26 (duraciones por pieza) y controles D8 con dt_s.

    Requiere una línea temporal común y cada robot estacionario en su meta al
    terminar; ambas condiciones son necesarias para una deformación CBS común.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'NPZ no encontrado: {path}')
    ctrl = {}
    durations = {}
    with np.load(path, allow_pickle=False) as z:
        for i, robot in enumerate(ROBOTS):
            ckey = f'R{i}_controls' if f'R{i}_controls' in z.files else f'R{i}'
            dkey = f'R{i}_durations_s'
            if ckey not in z.files:
                raise ValueError(f'{robot}: faltan controles Bézier')
            c = np.asarray(z[ckey], dtype=float)
            if dkey in z.files:
                d = np.asarray(z[dkey], dtype=float)
            elif 'dt_s' in z.files:
                d = np.full(len(c), float(z['dt_s']), dtype=float)
            else:
                raise ValueError(f'{robot}: faltan duraciones por segmento')
            if c.ndim != 3 or c.shape[1:] != (6, 2) or len(c) == 0:
                raise ValueError(f'{robot}: controles no tienen forma (N,6,2)')
            if d.shape != (len(c),) or not np.isfinite(d).all() or np.any(d <= 0):
                raise ValueError(f'{robot}: duraciones inválidas')
            if not np.isfinite(c).all():
                raise ValueError(f'{robot}: coordenadas no finitas')
            ctrl[robot], durations[robot] = c, d

    ref = durations['summit0']
    if any(d.shape != ref.shape or not np.allclose(d, ref, atol=1e-7, rtol=0)
           for d in durations.values()):
        raise ValueError('NOT_COMMON_TIMELINE: se requieren segmentos sincronizados')
    if len(ref) > 1:
        for robot, c in ctrl.items():
            d = durations[robot]
            dp = np.linalg.norm(c[:-1, -1] - c[1:, 0], axis=1)
            vl = 5 * (c[:-1, -1] - c[:-1, -2]) / d[:-1, None]
            vr = 5 * (c[1:, 1] - c[1:, 0]) / d[1:, None]
            al = 20 * (c[:-1, -1] - 2*c[:-1, -2] + c[:-1, -3]) / d[:-1, None]**2
            ar = 20 * (c[1:, 2] - 2*c[1:, 1] + c[1:, 0]) / d[1:, None]**2
            if (np.max(dp) > 1e-5 or np.max(np.linalg.norm(vl-vr, axis=1)) > 1e-4 or
                    np.max(np.linalg.norm(al-ar, axis=1)) > 1e-3):
                raise ValueError(f'{robot}: entrada Bézier no es C2 temporal')
    return ctrl, ref


def smoothstep(s):
    return s*s*s*(10 + s*(-15 + 6*s))


def smoothstep_prime(s):
    return 30*s*s*(1-s)*(1-s)


def scale_at(tau, baseline, ramps):
    q = baseline
    for r in ramps:
        if tau < r['start_tau']:
            break
        if tau >= r['end_tau']:
            q = r['to_scale']
        else:
            u = (tau-r['start_tau'])/(r['end_tau']-r['start_tau'])
            q = r['from_scale'] + (r['to_scale']-r['from_scale'])*smoothstep(u)
            break
    return float(q)


def max_qprime(a, b, ramps):
    """Cota exacta de la derivada positiva de smoothstep en [a,b]."""
    peak = 0.0
    for r in ramps:
        lo = max(a, r['start_tau'])
        hi = min(b, r['end_tau'])
        if hi <= lo:
            continue
        w = r['end_tau'] - r['start_tau']
        u = (lo-r['start_tau'])/w
        v = (hi-r['start_tau'])/w
        pts = [u, v] + ([0.5] if u <= 0.5 <= v else [])
        peak = max(peak, (r['to_scale']-r['from_scale'])/w *
                   max(smoothstep_prime(s) for s in pts))
    return peak


def segment_envelopes(ctrl, durations):
    v, acc = {}, {}
    for robot, c in ctrl.items():
        vc = 5 * np.diff(c, axis=1) / durations[:, None, None]
        ac = 20 * np.diff(c, n=2, axis=1) / durations[:, None, None]**2
        v[robot] = np.max(np.linalg.norm(vc, axis=2), axis=1)
        acc[robot] = np.max(np.linalg.norm(ac, axis=2), axis=1)
    return v, acc


def time_bounds(durations, v_env, a_env, baseline, ramps):
    """Cota global: Bernstein + regla de cadena para un reloj común creciente."""
    knots = np.r_[0., np.cumsum(durations)]
    vp = ap = 0.0
    witness = None
    for i in range(len(durations)):
        left, right = float(knots[i]), float(knots[i+1])
        q = scale_at(right, baseline, ramps)
        dq = max_qprime(left, right, ramps)
        for robot in ROBOTS:
            v = float(v_env[robot][i]) * q
            a = float(a_env[robot][i]) * q*q + float(v_env[robot][i])*q*dq
            if v > vp:
                vp = v
            if a > ap:
                ap, witness = a, {'robot': robot, 'segment_index': i}
    return vp, ap, witness


def stop_times(ctrl, durations, tol=1e-6):
    """Primer comienzo de segmento tras el cual el robot queda inmóvil."""
    knots = np.r_[0., np.cumsum(durations)]
    out = {}
    for robot, c in ctrl.items():
        goal = c[-1, -1]
        deviation = np.max(np.linalg.norm(c-goal[None, None, :], axis=2), axis=1)
        active = np.flatnonzero(deviation > tol)
        idx = int(active[-1] + 1) if len(active) else 0
        out[robot] = float(knots[idx])
    return out


def integrate_wall(a, b, baseline, ramps):
    """Integral dt = d(tau)/q(tau) por cuadratura Gauss-Legendre de orden 32."""
    if b <= a:
        return 0.0
    # Partir exactamente en comienzos y finales de rampas (función C2 suave).
    split = sorted({float(a), float(b)} | {
        float(p) for r in ramps for p in (r['start_tau'], r['end_tau']) if a < p < b
    })
    total = 0.0
    for left, right in zip(split[:-1], split[1:]):
        tau = (right+left)/2 + (right-left)/2 * _GAUSS_X
        inv = np.array([1/scale_at(float(x), baseline, ramps) for x in tau])
        total += float((right-left)/2 * np.dot(_GAUSS_W, inv))
    return total


def optimize_stage(path, max_speed=2., max_accel=2., max_scale=2., transition_tau=2.,
                   min_gain=1e-5):
    if not all(math.isfinite(x) and x > 0 for x in
               (max_speed, max_accel, max_scale, transition_tau)) or max_scale < 1:
        raise ValueError('Parámetros temporales fuera de rango')
    ctrl, durations = load_stage(path)
    v_env, a_env = segment_envelopes(ctrl, durations)
    orig_v = max(float(np.max(x)) for x in v_env.values())
    orig_a = max(float(np.max(x)) for x in a_env.values())
    if orig_v > max_speed + 1e-6 or orig_a > max_accel + 1e-6:
        raise ValueError(f'INPUT_EXCEEDS_LIMITS: envolvente v={orig_v:.5f}, a={orig_a:.5f}')
    k_v = math.inf if orig_v < 1e-10 else max_speed/orig_v
    k_a = math.inf if orig_a < 1e-10 else math.sqrt(max_accel/orig_a)
    baseline = min(max_scale, k_v, k_a)
    if not math.isfinite(baseline):
        baseline = max_scale
    baseline = max(1., baseline)
    horizon = float(np.sum(durations))
    finishes = stop_times(ctrl, durations)
    events = sorted({float(v) for v in finishes.values() if 1e-9 < v < horizon-1e-9})
    ramps = []
    attempts = []
    for ei, at in enumerate(events):
        next_event = events[ei+1] if ei+1 < len(events) else horizon
        end = min(at+transition_tau, next_event)
        if end-at < 1e-6:
            continue
        current = scale_at(at, baseline, ramps)
        if current >= max_scale-1e-8:
            attempts.append({'finish_tau': at, 'accepted': False, 'reason': 'MAX_SCALE_REACHED'})
            continue
        def feasible(target):
            test = ramps + [{'start_tau': at, 'end_tau': end,
                             'from_scale': current, 'to_scale': target}]
            vv, aa, _ = time_bounds(durations, v_env, a_env, baseline, test)
            return vv <= max_speed + 1e-8 and aa <= max_accel + 1e-8
        low, high = current, max_scale
        if feasible(high):
            low = high
        else:
            for _ in range(46):
                mid = (low+high)/2
                if feasible(mid):
                    low = mid
                else:
                    high = mid
        if low > current + min_gain:
            ramps.append({'start_tau': at, 'end_tau': end,
                          'from_scale': float(current), 'to_scale': float(low),
                          'robots_finished': [r for r, f in finishes.items() if abs(f-at)<1e-7]})
            attempts.append({'finish_tau': at, 'accepted': True,
                             'from_scale': float(current), 'to_scale': float(low)})
        else:
            attempts.append({'finish_tau': at, 'accepted': False,
                             'reason': 'NO_DYNAMICAL_HEADROOM'})
    bound_v, bound_a, witness = time_bounds(durations, v_env, a_env, baseline, ramps)
    if bound_v > max_speed+1e-6 or bound_a > max_accel+1e-6:
        raise RuntimeError('INTERNAL_CERTIFICATION_FAILED')
    uniform_time = horizon / baseline
    new_time = integrate_wall(0., horizon, baseline, ramps)
    if new_time > uniform_time + 1e-8:
        raise RuntimeError('INTERNAL_TIME_INCREASE')
    knots = sorted({0., horizon, *np.cumsum(durations).tolist(),
                    *finishes.values(), *[r['start_tau'] for r in ramps],
                    *[r['end_tau'] for r in ramps]})
    table = [{'tau_original_s': round(float(x), 10),
              'wall_time_s': round(integrate_wall(0., float(x), baseline, ramps), 10),
              'speed_factor': round(scale_at(float(x), baseline, ramps), 10)} for x in knots]
    return {
        'schema': 'swarm.d9929.collective_successive_timewarp.v1',
        'status': 'OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE',
        'execution_authorized': False,
        'input_npz': str(Path(path).resolve()), 'input_sha256': sha(path),
        'method': 'common_virtual_clock_with_C1_smooth_speed_factor',
        'max_speed_m_s': max_speed, 'max_accel_m_s2': max_accel,
        'max_scale': max_scale, 'transition_tau_s': transition_tau,
        'duration_original_s': horizon,
        'duration_uniform_s': uniform_time,
        'duration_successive_s': new_time,
        'saving_vs_uniform_s': max(0., uniform_time-new_time),
        'saving_vs_original_s': max(0., horizon-new_time),
        'uniform_scale': baseline,
        'finish_times_virtual_s': finishes,
        'finish_times_wall_s': {r: integrate_wall(0., v, baseline, ramps) for r, v in finishes.items()},
        'ramps': ramps, 'attempts': attempts,
        'certified_envelopes': {'speed_m_s': bound_v, 'accel_m_s2': bound_a,
                                'accel_witness': witness,
                                'method': 'quintic_Bernstein_convex_hull_and_chain_rule'},
        'common_time_warp': True,
        'collision_preservation': 'Preserva la coincidencia de tiempos entre robots; requiere entrada CBS válida',
        'source_C2_checked': True,
        'timewarp_knots': table,
        'limitations': [
            'Horario/ley de reloj virtual offline; NO es aún un NPZ con duración constante por pieza',
            'La ejecución requerirá un seguidor que evalúe el reloj común sin alterar la geometría ni la sincronización',
            'No certifica de nuevo las rutas CBS de origen ni el agarre físico',
        ],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    inp = p.add_mutually_exclusive_group(required=True)
    inp.add_argument('--trajectory', type=Path)
    inp.add_argument('--timed-dir', type=Path)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--max-speed', type=float, default=2.)
    p.add_argument('--max-accel', type=float, default=2.)
    p.add_argument('--max-scale', type=float, default=2.)
    p.add_argument('--transition-tau', type=float, default=2.,
                   help='Duración de la rampa suave en segundos VIRTUALES')
    args = p.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        p.error('Carpeta de salida no vacía: no sobrescribimos otros resultados')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    paths = ({'stage': args.trajectory} if args.trajectory else {
        stage: args.timed_dir/f'{stage}_MILP_TIMED_VALIDATED_OFFLINE.npz'
        for stage in STAGES})
    summary = {'schema': 'swarm.d9929.timewarp_batch.v1', 'status': 'RUNNING',
               'execution_authorized': False, 'stages': {}, 'problems': []}
    _write_json(args.output_dir/'report.json', summary)
    for stage, path in paths.items():
        try:
            record = optimize_stage(path, args.max_speed, args.max_accel,
                                    args.max_scale, args.transition_tau)
            _write_json(args.output_dir/f'{stage}_clock.json', record)
            summary['stages'][stage] = {
                'duration_original_s': record['duration_original_s'],
                'duration_uniform_s': record['duration_uniform_s'],
                'duration_successive_s': record['duration_successive_s'],
                'saving_vs_uniform_s': record['saving_vs_uniform_s'],
                'ramps': len(record['ramps']), 'status': record['status'],
                'clock_file': str(args.output_dir/f'{stage}_clock.json'),
            }
            print(f'{stage}: antes {record["duration_original_s"]:.3f}s, '
                  f'uniforme {record["duration_uniform_s"]:.3f}s, '
                  f'sucesivo {record["duration_successive_s"]:.3f}s '
                  f'(ganancia vs uniforme {record["saving_vs_uniform_s"]:.3f}s)', flush=True)
        except Exception as exc:
            summary['problems'].append(f'{stage}: {type(exc).__name__}: {exc}')
        _write_json(args.output_dir/'report.json', summary)
    summary['status'] = 'OFFLINE_CLOCKS_CERTIFIED' if not summary['problems'] else 'PARTIAL_OR_REJECTED'
    _write_json(args.output_dir/'report.json', summary)
    print('Reporte:', args.output_dir/'report.json')
    return 0 if not summary['problems'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

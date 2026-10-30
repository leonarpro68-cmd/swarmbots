#!/usr/bin/env python3
"""D9.7A: independent-time holonomic follower core (NO ROS publishers).

The reference is the D9.5 certified D9.4 Bézier trajectory. All motion
commands are HYPOTHETICAL until a separate, reviewed ROS actuator is attached.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')


def finite(*numbers):
    return all(isinstance(v, (float, int)) and math.isfinite(v) for v in numbers)


def calculate_command(reference, observation, previous=(0.0, 0.0), dt=0.02,
                      kp=0.8, vmax=1.2, amax=1.0):
    """World XY feedforward+feedback into body-frame planar Twist command.

    Rate limiting is in body frame; caller must account for body rotation and
    apply robot-velocity/acceleration safety monitoring before any actuation.
    """
    rx, ry = reference['x_m'], reference['y_m']
    vx, vy = reference['vx_m_s'], reference['vy_m_s']
    x, y, yaw = observation['x_m'], observation['y_m'], observation['yaw_rad']
    if not finite(rx, ry, vx, vy, x, y, yaw, *previous, dt, kp, vmax, amax):
        raise ValueError('Nonfinite command input')
    if not (0 < dt <= 0.25 and 0 <= kp <= 10 and 0 < vmax <= 1.2 and 0 < amax <= 1.0):
        raise ValueError('Out-of-policy controller parameter')
    ex, ey = rx-x, ry-y
    wx, wy = vx+kp*ex, vy+kp*ey
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = c*wx+s*wy, -s*wx+c*wy
    speed = math.hypot(bx, by)
    if speed > vmax:
        bx, by = bx*vmax/speed, by*vmax/speed
    dx, dy = bx-previous[0], by-previous[1]
    increment = math.hypot(dx, dy)
    if increment > amax*dt:
        bx = previous[0] + dx*amax*dt/increment
        by = previous[1] + dy*amax*dt/increment
    return {'vx_body_m_s': bx, 'vy_body_m_s': by,
            'position_error_m': math.hypot(ex, ey),
            'command_speed_m_s': math.hypot(bx, by)}


def verify_observations(reference, observations, *, max_error=0.6, min_sep=1.15):
    """All robots pass together, or every proposed command must be discarded."""
    if set(reference) != set(ROBOTS) or set(observations) != set(ROBOTS):
        return ['MISSING_ROBOT']
    issues=[]
    for robot in ROBOTS:
        r, o = reference[robot], observations[robot]
        vals=[r['x_m'],r['y_m'],o['x_m'],o['y_m'],o['yaw_rad']]
        if not finite(*vals):
            issues.append(robot+':NONFINITE'); continue
        if math.hypot(r['x_m']-o['x_m'],r['y_m']-o['y_m']) > max_error:
            issues.append(robot+':TRACKING_ERROR')
        if o.get('frame_id') != robot+'/odom' or o.get('child_frame_id') != robot+'/base_footprint':
            issues.append(robot+':FRAME_MISMATCH')
    for i, ra in enumerate(ROBOTS):
        for rb in ROBOTS[i+1:]:
            a,b=observations[ra],observations[rb]
            if all(finite(o['x_m'],o['y_m']) for o in (a,b)) and math.dist((a['x_m'],a['y_m']), (b['x_m'],b['y_m'])) < min_sep:
                issues.append(ra+'-'+rb+':SEPARATION')
    return issues


def preview(run:Path,stage:str,step:float=0.02):
    from d96a_readonly_follower import load_stage, sample
    if not 0 < step <= .05:
        raise ValueError('step must be (0,0.05]')
    manifest, traj = load_stage(run,stage)
    if manifest.get('status')!='PASS_READ_ONLY':
        raise ValueError('D9.5 manifest not accepted')
    horizon=max(float(durations.sum()) for _,durations in traj.values())
    t=0.0; prev={r:(0.,0.) for r in ROBOTS}; count=0; peaks={r:0. for r in ROBOTS}
    while t <= horizon+1e-9:
        refs=sample(traj,min(t,horizon))
        # Perfect-tracking-only preview tests time handling and kinematic frame conversion.
        for r in ROBOTS:
            o={'x_m':refs[r]['x_m'],'y_m':refs[r]['y_m'],'yaw_rad':0.0}
            command=calculate_command(refs[r],o,prev[r],step)
            prev[r]=(command['vx_body_m_s'],command['vy_body_m_s'])
            peaks[r]=max(peaks[r],command['command_speed_m_s'])
        t+=step;count+=1
    report={'schema':'swarm.d97.core_preview.v1','status':'PASS_CORE_PREVIEW_ONLY',
            'stage':stage,'horizon_s':horizon,'samples':count,
            'robot_durations_s':{r:float(traj[r][1].sum()) for r in ROBOTS},
            'hypothetical_peak_command_speeds_m_s':peaks,
            'commands_sent':0,'publishers_created':0,'execution_authorized':False,
            'note':'Idealized perfect tracking; not a dynamic response simulation or actuator test.'}
    out=run/'d97_core';out.mkdir(parents=True,exist_ok=True)
    (out/f'{stage}_core_preview.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',default='pickup1')
    p.add_argument('--step',type=float,default=0.02)
    a=p.parse_args()
    print(json.dumps(preview(a.run,a.stage,a.step),indent=2))

if __name__=='__main__': main()

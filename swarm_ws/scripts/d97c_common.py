"""D97C shared pure validation. Simulation only."""
import math
ROBOTS=tuple(f'summit{i}' for i in range(3))

def clamp_xy(x,y,cap):
    if not all(map(math.isfinite,(x,y,cap))) or cap<=0: raise ValueError('invalid velocity')
    s=math.hypot(x,y)
    return (x,y) if s<=cap else (x*cap/s,y*cap/s)

def gate(observations, refs, *, start=False, tol=.08, track=.50, sep=1.15, age=.30):
    if set(observations)!=set(ROBOTS) or set(refs)!=set(ROBOTS):return ['MISSING_ROBOT']
    errors=[]
    for name in ROBOTS:
        o,r=observations[name],refs[name]
        if o.get('frame_id') != name+'/odom' or o.get('child_frame_id') != name+'/base_footprint':errors.append(name+':FRAME')
        if o.get('age',1e9)>age: errors.append(name+':STALE')
        vals=[o.get('x'),o.get('y'),o.get('yaw'),o.get('speed'),r.get('x_m'),r.get('y_m')]
        if any(v is None or not math.isfinite(v) for v in vals):errors.append(name+':INVALID');continue
        dist=math.hypot(o['x']-r['x_m'],o['y']-r['y_m'])
        if dist>(tol if start else track):errors.append(name+':POSITION')
        if start and o['speed']>.03:errors.append(name+':MOVING')
        if not start and o['speed']>1.35:errors.append(name+':SPEED')
    for i,a in enumerate(ROBOTS):
        for b in ROBOTS[i+1:]:
            if all(math.isfinite(observations[r].get(k,float('nan'))) for r in (a,b) for k in ('x','y')):
                if math.dist((observations[a]['x'],observations[a]['y']),(observations[b]['x'],observations[b]['y']))<sep:
                    errors.append(a+'-'+b+':SEPARATION')
    return errors

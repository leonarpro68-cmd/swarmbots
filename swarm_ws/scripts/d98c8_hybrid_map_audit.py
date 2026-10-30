#!/usr/bin/env python3
"""D9.8C.8 offline swept-geometry map audit; never certifies Hybrid V2 or authorizes execution."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
import yaml
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
PHASES=('wave1','clear_deposit','wave2')
ROBOTS=('summit0','summit1','summit2')

def bezier(c,t):
    a=np.asarray(c,dtype=float).copy()
    for j in range(5,0,-1):a=(1-t)*a[:-1]+t*a[1:]
    return a[0]

def scene(raw):
    ext=raw.get('extent')
    if not isinstance(ext,list) or len(ext)!=4 or not all(math.isfinite(float(x)) for x in ext):raise ValueError('INVALID_MAP_EXTENT')
    boundary=box(*ext)
    polys=[]
    for k,obs in enumerate(raw.get('obstacles',[])):
        poly=Polygon(obs['outer'],holes=obs.get('holes') or [])
        if poly.is_empty or not poly.is_valid:raise ValueError(f'INVALID_OBSTACLE_{k}')
        polys.append(poly)
    return boundary,unary_union(polys) if polys else None

def audit(preview,retimed,controls,vector_map,radius=.6,step=.04):
    if preview.get('status')!='BEZIER_PREVIEW_LIMITS_PASS_NOT_EXECUTABLE' or retimed.get('status')!='D92_PHASE_MODEL_THREE_PHASES_PASS_NOT_EXECUTABLE':raise ValueError('UNCERTIFIED_UPSTREAM_PREVIEW')
    if retimed.get('execution_authorized') is not False or not (0<step<=.1 and 0<radius<=2):raise ValueError('UNSAFE_PARAMETERS')
    boundary,obstacles=scene(vector_map)
    report=[]
    for phase,pr in zip(PHASES,retimed['phases']):
        if pr.get('phase')!=phase or pr.get('status')!='D92_CONSERVATIVE_PHASE_MODEL_PASS':raise ValueError('PHASE_MISMATCH')
        robots={}
        for r in ROBOTS:
            key=f'{phase}_{r}'; cs=np.asarray(controls[f'{key}_controls'],dtype=float); ds=np.asarray(controls[f'{key}_durations_s'],dtype=float)
            if cs.ndim!=3 or cs.shape[1:]!=(6,2) or ds.shape!=(len(cs),) or np.any(ds<=0) or not np.isfinite(cs).all():raise ValueError('INVALID_CONTROLS_'+key)
            # For a cubic/quintic Bezier curve, all positions stay in the convex hull of its control points.
            # Requiring *every* point in the bounding disk to clear obstacles and world extent
            # is conservative, even if control-point hull overapproximates the real path.
            issues=[];closest=float('inf'); outside=[]
            for i,c in enumerate(cs):
                # Conservative convex hull (not just sampled centerline).
                from shapely.geometry import MultiPoint
                hull=MultiPoint([tuple(v) for v in c]).convex_hull.buffer(radius)
                if not boundary.covers(hull):outside.append(i)
                if obstacles is not None:
                    clearance=hull.distance(obstacles)
                    closest=min(closest,clearance)
                    if hull.intersects(obstacles):issues.append(i)
            robots[r]={'pieces':len(cs),'outside_extent_piece_indices':outside[:30], 'obstacle_intersection_piece_indices':issues[:30], 'obstacle_clearance_lower_bound_m':round(closest,5) if math.isfinite(closest) else None, 'conservative_hull_check_pass':not(outside or issues)}
        report.append({'phase':phase,'robots':robots,'map_hulls_pass':all(v['conservative_hull_check_pass'] for v in robots.values())})
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preview',type=Path,required=True)
    p.add_argument('--retimed',type=Path,required=True)
    p.add_argument('--map-vector',type=Path,required=True,help='Explicitly chosen vector map matching the actual D8 scenario')
    p.add_argument('--npz',type=Path)
    p.add_argument('--radius',type=float,default=.6)
    p.add_argument('--output',type=Path)
    a=p.parse_args(); npz_path=a.npz or a.preview.with_suffix('.npz')
    prev=json.loads(a.preview.read_text());ret=json.loads(a.retimed.read_text());raw=yaml.safe_load(a.map_vector.read_text()); sha=hashlib.sha256(npz_path.read_bytes()).hexdigest()
    if ret.get('source_sha256')!=sha:raise SystemExit('ERROR: NPZ_SHA_MISMATCH')
    with np.load(npz_path,allow_pickle=False) as controls: phases=audit(prev,ret,controls,raw,a.radius)
    good=all(v['map_hulls_pass'] for v in phases)
    out={'schema':'swarm.d98c8.hybrid_map_screen.v1','status':'MAP_HULL_SCREEN_PASS_NOT_HYBRID_CERTIFIED' if good else 'BLOCKED_MAP_GEOMETRY',
         'map_file':str(a.map_vector),'map_extent':raw['extent'],'robot_radius_conservative_m':a.radius,'npz_sha256':sha,
         'phases':phases,'hybrid_v2_certified':False,'full_obstacle_certified':False,'execution_authorized':False,'commands_sent':0,
         'limitations':['Map hull test is conservative: false positives possible.','Does not invoke Hybrid V2 planner or its occupancy/collision certificate.',
         'Does not validate load attachment, fresh odometry, physical release, or D9.4/D9.5.']}
    dest=a.output or a.preview.parent/'delivery1_d98c8_hybrid_map_audit.json';dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':out['status'],'output':str(dest),'per_phase':[{'phase':x['phase'],'map_hulls_pass':x['map_hulls_pass']} for x in phases]},indent=2))
    return 0 if good else 2
if __name__=='__main__':raise SystemExit(main())

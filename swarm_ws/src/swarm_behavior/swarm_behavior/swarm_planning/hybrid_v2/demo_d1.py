"""D.1 smoke test of the REAL SwarmBots CBS => Hybrid V2 input adapter (offline)."""
from __future__ import annotations
import argparse,json
from pathlib import Path
from ..bridge import PlanningBridge
from ..scenario import generate, load
from .. import geometry as geom
from .cbs_adapter import adapt_plan,plan_v2


def initial_starts(graph):
    vs=sorted(graph.vertices)
    return (vs[0],vs[len(vs)//2],vs[-1])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,default=123)
    ap.add_argument('--scenario',type=Path,help='Optional real scenario JSON')
    ap.add_argument('--solve',action='store_true',help='Run SLSQP C.2 and its certificate (may reject)')
    ap.add_argument('--iterations',type=int,default=2)
    ap.add_argument('--margin',type=float,default=.05)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    bridge=PlanningBridge(load(a.scenario) if a.scenario else generate(a.seed))
    starts=initial_starts(bridge.graph)
    plan=bridge.plan_pickup(starts)
    data=adapt_plan(plan,bridge.graph,stage='pickup1')
    report={'stage':'pickup1','seed':a.seed,'starts':starts,'cbs_lengths':[len(p) for p in data.paths],
            'shape':list(data.points.shape),'obstacle_count':len(data.obstacles),
            'pending_trash':list(data.pending_trash),
            'note':'D.1 adapter input verified, not motion certified'}
    if a.solve:
        result, _=plan_v2(plan,bridge.graph,stage='pickup1',margin=a.margin,iterations=a.iterations)
        report['hybrid_v2']=result
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,default=str))
    print(json.dumps(report,indent=2,ensure_ascii=False,default=str))
    return 0

if __name__=='__main__':
    raise SystemExit(main())

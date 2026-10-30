import json
import os
import numpy as np
import pytest
from swarm_behavior.swarm_planning.hybrid_v2.scenario import reference_inputs
from swarm_behavior.swarm_planning.hybrid_v2.slsqp import solve_all
from swarm_behavior.swarm_planning.hybrid_v2.regression_b import load_controls, diagnostics, compare


def test_solver_without_global_nodes():
    paths=[[(0,0),(1,0),(2,0)]]
    pts=np.array([[[0.,0.],[1.,0.],[2.,0.]]])
    square=[(np.array([1.,0.]),100.),(np.array([-1.,0.]),100.),
            (np.array([0.,1.]),100.),(np.array([0.,-1.]),100.)]
    paths_out,stats=solve_all(pts,paths,[[square,square]],maxiter=120)
    assert len(paths_out)==1 and paths_out[0].shape==(2,6,2)
    assert stats[0]['success']
    assert stats[0]['inequality_violation']<1e-6
    assert stats[0]['equality_violation']<1e-6


def test_reference_saved_outputs_when_provided():
    folder=os.getenv('HYBRID_V2_REFERENCE_DIR')
    if not folder:pytest.skip('Set HYBRID_V2_REFERENCE_DIR for original artifacts')
    from pathlib import Path
    d=Path(folder)
    expected=json.loads((d/'reporte.json').read_text())
    p,pts,obs=reference_inputs()
    initial=diagnostics(load_controls(d/'controles_slsqp.npz'),p,pts,obs)
    final=diagnostics(load_controls(d/'controles_slsqp_refinados.npz'),p,pts,obs)
    compare({'initial':initial,'final':final},expected)
    assert all(v['C2_max'] < 1e-7 for v in final['continuity'].values())
    assert min(final['sampled_obstacle_clearance_m'])>=-1e-5
    assert min(final['sampled_pair_clearance_m'].values())>=-1e-5

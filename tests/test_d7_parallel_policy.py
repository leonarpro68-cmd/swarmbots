"""Pure scheduling/safety checks. Does not launch ROS or Gazebo."""
import json
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'swarm_ws'/'scripts'))
from d7_parallel_delivery import plan_release_waves, segment_distance

ROBOTS=['summit0','summit1','summit2']

def plan(tmp_path, colors, starts, dep_xy, clearance=1.25):
    assignment={r:f'trash_{i}_{colors[i]}' for i,r in enumerate(ROBOTS)}
    trash={n:{'color':c} for n,c in [(assignment[r],colors[i]) for i,r in enumerate(ROBOTS)]}
    dep={color:'deposit_'+color for color in dep_xy}
    scenario=tmp_path/'scenario.json'
    scenario.write_text(json.dumps({'elements':[{'name':name,'type':'deposit','xy_m':xy} for color,xy in dep_xy.items() for name in ['deposit_'+color]]}))
    t={'status':'arrived_not_delivered','carried':assignment,'reference_xy':dict(zip(ROBOTS,starts))}
    return plan_release_waves(scenario=scenario, assignments=assignment, trash=trash,
                               deposits=dep, transport=t, max_start_distance=1.80,clearance_m=clearance)

def test_three_distinct_deposits_one_wave(tmp_path):
    wave,*_=plan(tmp_path,['verde','naranja','azul'],[(0,0),(0,5),(0,10)],
                 {'verde':(1.5,0),'naranja':(1.5,5),'azul':(1.5,10)})
    assert wave==[['summit0','summit1','summit2']]

def test_two_shared_deposit_two_waves(tmp_path):
    wave,*_=plan(tmp_path,['verde','verde','naranja'],[(0,0),(3,0),(0,6)],
                 {'verde':(1.5,0),'naranja':(1.5,6)})
    assert len(wave)==2
    assert sorted(map(len,wave))==[1,2]
    assert 'summit2' in wave[0]
    assert sorted(sum(wave,[]))==sorted(ROBOTS)
    assert not {'summit0','summit1'}.issubset(wave[0])

def test_three_shared_deposit_three_waves(tmp_path):
    # three positions 1.5m from deposit, well separated from each other
    wave,*_=plan(tmp_path,['verde']*3,[(0,0),(3,0),(1.5,1.5)],{'verde':(1.5,0)})
    assert len(wave)==3
    assert all(len(w)==1 for w in wave)

def test_invalid_carrying_state_refuses(tmp_path):
    assignment={r:f'trash_{i}_verde' for i,r in enumerate(ROBOTS)}
    scenario=tmp_path/'scenario.json'; scenario.write_text('{"elements":[]}')
    with pytest.raises(RuntimeError,match='Estado de transporte'):
        plan_release_waves(scenario=scenario,assignments=assignment,
                           trash={t:{'color':'verde'} for t in assignment.values()},
                           deposits={'verde':'deposit_verde'},
                           transport={'status':'wrong','carried':assignment,'reference_xy':{}})

def test_outside_local_start_range_refuses(tmp_path):
    with pytest.raises(RuntimeError,match='fuera del preflight'):
        plan(tmp_path,['verde','naranja','azul'],[(-3,0),(0,5),(0,10)],
             {'verde':(1.5,0),'naranja':(1.5,5),'azul':(1.5,10)})

def test_segment_intersection_must_block_parallel_pickup():
    assert segment_distance((0,0),(2,2),(0,2),(2,0))==0


def test_coordinator_calls_original_parallel_pickups_and_barrier():
    source=(Path(__file__).resolve().parents[1]/'swarm_ws/scripts/run_hybrid_v2_mission_d7.py').read_text()
    assert 'parallel_grasps(' in source
    assert 'release_waves(' in source
    assert 'on_wave_complete=after_wave' in source
    assert "if set(receipts)!=set(assignments.items()):" in source

def test_wave_executor_waits_for_all_before_next_wave(tmp_path):
    import time
    from d7_parallel_delivery import release_waves
    colors=['verde','verde','naranja']
    assignments={r:f'trash_{i}_{colors[i]}' for i,r in enumerate(ROBOTS)}
    trash={assignments[r]:{'color':colors[i]} for i,r in enumerate(ROBOTS)}
    dep={'verde':'deposit_verde','naranja':'deposit_naranja'}
    scenario=tmp_path/'world.json'
    scenario.write_text(json.dumps({'elements':[
        {'type':'deposit','name':'deposit_verde','xy_m':[1.5,0]},
        {'type':'deposit','name':'deposit_naranja','xy_m':[1.5,6]},
    ]}))
    fake=tmp_path/'fake_release.py'
    fake.write_text('''import sys,time
print('FAKE_START',time.monotonic(),flush=True)
time.sleep(0.12)
print('DETACH CONFIRMADO',flush=True)
print('REGRESO CONFIRMADO',flush=True)
''')
    transport={'status':'arrived_not_delivered','carried':assignments,'reference_xy':{
        'summit0':[0,0],'summit1':[3,0],'summit2':[0,6],
    }}
    receipts=[]; barriers=[]
    release_waves(scenario=scenario,assignments=assignments,trash=trash,deposits=dep,
                  transport=transport,script=fake,logfile=tmp_path/'waves.log',
                  state_callback=lambda r,t:receipts.append((r,t)),
                  on_wave_complete=lambda w:barriers.append(tuple(w)),
                  max_start_distance=1.8,timeout_s=10)
    assert len(receipts)==3
    assert len(barriers)==2
    assert len(barriers[0])==2
    assert len(barriers[1])==1
    log=(tmp_path/'waves.log').read_text()
    assert log.count('FAKE_START')==3
    assert log.index('### OLEADA 2')>log.index('REGRESO CONFIRMADO')


def test_wave_executor_stops_on_missing_confirmation(tmp_path):
    from d7_parallel_delivery import release_waves
    assignments={r:f'trash_{i}_{color}' for i,(r,color) in enumerate(zip(ROBOTS,['verde','naranja','azul']))}
    trash={name:{'color':name.rsplit('_',1)[-1]} for name in assignments.values()}
    dep={c:'deposit_'+c for c in ['verde','naranja','azul']}
    scenario=tmp_path/'world.json'
    scenario.write_text(json.dumps({'elements':[{'name':'deposit_'+c,'type':'deposit','xy_m':[1.5,5*i]}
                                  for i,c in enumerate(['verde','naranja','azul'])]}))
    fake=tmp_path/'fail_release.py'
    fake.write_text("print('DETACH CONFIRMADO')\n# return success without REGRESO, executor must reject\n")
    trans={'status':'arrived_not_delivered','carried':assignments,'reference_xy':{r:[0,5*i] for i,r in enumerate(ROBOTS)}}
    with pytest.raises(RuntimeError,match='ACK faltantes'):
        release_waves(scenario=scenario,assignments=assignments,trash=trash,deposits=dep,
                      transport=trans,script=fake,logfile=tmp_path/'failure.log',
                      state_callback=lambda r,t:None,timeout_s=8,max_start_distance=1.8)

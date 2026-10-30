import json
import numpy as np
import pytest
from swarm_behavior.swarm_planning.hybrid_v2.ros_protocol_adapter import build_commands
from swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol import decode_command


def fixture(tmp_path,certified=True):
    pts=np.zeros((2,6,2),dtype=float)
    np.savez_compressed(tmp_path/'stage.npz',R0=pts,R1=pts,R2=pts,dt_s=2.)
    r={'stages':{'pickup1':{'certified':certified,'hybrid_v2':{'certified':certified,'dt_s':2.,'pieces':2},'controls_file':str(tmp_path/'stage.npz')}}}
    (tmp_path/'report.json').write_text(json.dumps(r))
    return tmp_path/'report.json'


def test_roundtrip_three_robots(tmp_path):
    f=fixture(tmp_path)
    out=build_commands(f,'pickup1',session_id='test',start_time_s=123.,frame_ids=['odom']*3)
    assert len(out)==3
    assert all(decode_command(s).duration_s==4. for s in out.values())


def test_reject_uncertified(tmp_path):
    f=fixture(tmp_path,False)
    with pytest.raises(ValueError,match='certificación'):
        build_commands(f,'pickup1',session_id='test',start_time_s=123.,frame_ids=['odom']*3)


def test_reject_mismatch(tmp_path):
    f=fixture(tmp_path)
    data=json.loads(f.read_text());data['stages']['pickup1']['hybrid_v2']['dt_s']=1.
    f.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='dt_s'):
        build_commands(f,'pickup1',session_id='test',start_time_s=123.,frame_ids=['odom']*3)

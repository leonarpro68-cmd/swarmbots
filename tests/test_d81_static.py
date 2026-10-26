from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
s=(ROOT/'swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py').read_text()
launch=(ROOT/'swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py').read_text()
def test_timeout():
    assert '"attach_timeout", 5.0' in s
    assert "'attach_timeout': 5.0" in launch
def test_unknown_blocks():
    assert 'self._uncertain_joint = ("attach", failed)' in s
    assert 'self._uncertain_joint = ("detach", name)' in s
    assert 'AGARRE BLOQUEADO:' in s and 'LIBERACION BLOQUEADA:' in s
def test_late_ack_observed():
    assert 'GZ_V2 ACK TARDIO' in s
    assert 'if name is not None and self._uncertain_joint is None' in s
def test_shutdown_guard():
    assert 'if rclpy.ok():\n            rclpy.shutdown()' in s

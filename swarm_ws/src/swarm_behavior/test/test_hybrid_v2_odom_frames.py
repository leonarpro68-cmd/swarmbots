"""Frame-name guards for the Gazebo-world odometry experiment."""
import importlib.util
from pathlib import Path
import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'replan_hybrid_v2_from_odom.py'
spec = importlib.util.spec_from_file_location('v2_live_frames', SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def samples(frames):
    return {name: {'frame': frame} for name, frame in frames.items()}


def test_namespaced_frames_accepted_only_with_opt_in():
    odoms = samples({f'summit{i}': f'summit{i}/odom' for i in range(3)})
    assert mod.validate_odom_frames(odoms, accept_namespaced_world_odom=True) == {
        f'summit{i}': f'summit{i}/odom' for i in range(3)}
    with pytest.raises(ValueError):
        mod.validate_odom_frames(odoms, expected_frame='odom')


def test_wrong_robot_frame_is_rejected():
    odoms = samples({'summit0':'summit1/odom', 'summit1':'summit1/odom', 'summit2':'summit2/odom'})
    with pytest.raises(ValueError, match='summit0'):
        mod.validate_odom_frames(odoms, accept_namespaced_world_odom=True)


def test_shared_frame_backward_compatible_and_no_implicit_fallback():
    odoms = samples({f'summit{i}': 'map' for i in range(3)})
    assert len(mod.validate_odom_frames(odoms, expected_frame='map')) == 3
    with pytest.raises(ValueError, match='Specify'):
        mod.validate_odom_frames(odoms)

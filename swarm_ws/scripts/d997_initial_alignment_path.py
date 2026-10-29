#!/usr/bin/env python3
"""D9.9.7: align historical Bézier starts via EXISTING /summitN/path_follower.

Dry-run by default. --execute publishes nav_msgs/Path ONLY, never cmd_vel.
Use the provided d997_alignment_world.launch.py with 0.02m waypoint tolerance.
This step DOES NOT launch the timed trajectory or certify collision freedom.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

ROBOTS = tuple(f'summit{i}' for i in range(3))


def starts_from_npz(path):
    with np.load(path, allow_pickle=False) as z:
        starts = {}
        for i, name in enumerate(ROBOTS):
            pts = z[f'R{i}_controls']
            if pts.ndim != 3 or pts.shape[1:] != (6, 2) or not len(pts) or not np.isfinite(pts).all():
                raise ValueError(f'Invalid controls: {name}')
            starts[name] = tuple(map(float, pts[0, 0]))
    return starts


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--execute', action='store_true', help='Send short alignment paths to existing followers')
    p.add_argument('--position-tolerance', type=float, default=.03)
    p.add_argument('--max-offset', type=float, default=.30)
    p.add_argument('--stationary-speed', type=float, default=.03)
    p.add_argument('--settle-seconds', type=float, default=1.0)
    p.add_argument('--timeout', type=float, default=30.0)
    p.add_argument('--recovery-seconds', type=float, default=3.0,
                   help='Maximum wall seconds to recover fresh clock/odom after a safe stop')
    p.add_argument('--max-recoveries', type=int, default=2,
                   help='Maximum number of recovery cycles before rejecting')
    p.add_argument('--discovery-timeout', type=float, default=12.0,
                   help='Wait for ROS graph discovery of existing path_follower publishers')
    a = p.parse_args()
    if not (.02 <= a.position_tolerance <= .05 and 0 < a.stationary_speed <= .1
            and .05 <= a.max_offset <= .5 and a.settle_seconds > .2 and a.timeout > 2
            and 1 <= a.discovery_timeout <= 45 and 0.5 <= a.recovery_seconds <= 10
            and 0 <= a.max_recoveries <= 5):
        p.error('Unsafe thresholds')
    targets = starts_from_npz(a.trajectory)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node('d997_initial_alignment_path_supervisor', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    odom, clock = {}, []
    subs = []
    for name in ROBOTS:
        subs.append(node.create_subscription(Odometry, f'/{name}/odom',
                                             lambda msg, r=name: odom.__setitem__(r, (msg, time.monotonic())),
                                             qos_profile_sensor_data))
    subs.append(node.create_subscription(Clock, '/clock', lambda m: clock.append((m.clock.sec + m.clock.nanosec*1e-9, time.monotonic())), qos_profile_sensor_data))
    pubs = {r: node.create_publisher(RosPath, f'/{r}/planned_path', 10) for r in ROBOTS} if a.execute else {}
    problems, state, initial, final, moved = [], 'REJECT', {}, {}, []
    graph_diagnostics = {}
    graph_wait_s = 0.0
    recovery_events = []

    def snapshot_detailed():
        now = time.monotonic()
        if len(clock) < 2:
            return None, 'CLOCK_SAMPLES_MISSING'
        if now - clock[-1][1] > .75:
            return None, f'CLOCK_STALE_WALL_{now-clock[-1][1]:.3f}s'
        if clock[-1][0] - clock[0][0] < .05:
            return None, 'CLOCK_NOT_ADVANCING'
        result = {}
        for r in ROBOTS:
            if r not in odom:
                return None, f'ODOM_MISSING_{r}'
            if now - odom[r][1] > .75:
                return None, f'ODOM_STALE_{r}_{now-odom[r][1]:.3f}s'
            m = odom[r][0]
            if m.header.frame_id != f'{r}/odom' or m.child_frame_id != f'{r}/base_footprint':
                return None, f'FRAME_MISMATCH_{r}'
            xy = (float(m.pose.pose.position.x), float(m.pose.pose.position.y))
            v = m.twist.twist.linear
            stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            if not all(math.isfinite(value) for value in (*xy, v.x, v.y, stamp)):
                return None, f'INVALID_ODOMETRY_{r}'
            if abs(clock[-1][0]-stamp) > .25:
                return None, f'CLOCK_ODOM_STAMP_SKEW_{r}_{clock[-1][0]-stamp:+.3f}s'
            result[r] = {'xy': xy, 'error_m': math.dist(xy, targets[r]),
                         'speed_m_s': math.hypot(v.x, v.y)}
        return result, None

    def snapshot():
        return snapshot_detailed()[0]

    def send_path(r, xy):
        msg = RosPath()
        msg.header.frame_id = f'{r}/odom'
        msg.header.stamp = node.get_clock().now().to_msg()
        for pos in xy:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = pos
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        pubs[r].publish(msg)

    def cancel_all():
        if not a.execute:
            return
        for _ in range(3):
            for r in moved:
                send_path(r, [])
            rclpy.spin_once(node, timeout_sec=.08)

    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            initial = snapshot()
            if initial:
                break
        if not initial:
            raise RuntimeError('Clock/odometry missing, stale, or frame mismatch')
        if any(x['speed_m_s'] > a.stationary_speed for x in initial.values()):
            raise RuntimeError('One or more robots already moving')
        if any(x['error_m'] > a.max_offset for x in initial.values()):
            raise RuntimeError('Offset exceeds conservative alignment limit')
        moved = [r for r in ROBOTS if initial[r]['error_m'] > a.position_tolerance]
        # The ROS graph is discovered asynchronously. A single immediate query
        # can return [] even when a valid follower is starting up. Never skip
        # this ownership check; instead wait for exactly one known publisher.
        graph_start = time.monotonic()
        while True:
            graph_diagnostics = {}
            for r in ROBOTS:
                owners = node.get_publishers_info_by_topic(f'/{r}/cmd_vel')
                graph_diagnostics[r] = [
                    {'node_namespace': x.node_namespace, 'node_name': x.node_name}
                    for x in owners
                ]
            ready = all(
                len(graph_diagnostics[r]) == 1 and
                graph_diagnostics[r][0]['node_name'] == 'path_follower' and
                graph_diagnostics[r][0]['node_namespace'] == f'/{r}'
                for r in ROBOTS
            )
            graph_wait_s = time.monotonic() - graph_start
            if ready:
                break
            if graph_wait_s >= a.discovery_timeout:
                nodes = sorted(f'{ns.rstrip("/")}/{name}'
                               for name, ns in node.get_node_names_and_namespaces()
                               if name == 'path_follower')
                raise RuntimeError(
                    f'No se descubrieron exactamente los 3 seguidores tras '
                    f'{graph_wait_s:.1f}s: cmd_vel publishers={graph_diagnostics}; '
                    f'path_follower nodes visibles={nodes}. '
                    'Verifica el launch y ejecuta ros2 topic info '
                    '/summit0/cmd_vel --verbose')
            rclpy.spin_once(node, timeout_sec=.15)

        # Recheck pose/velocity after discovery: no stale pre-wait snapshot
        # may be used to start physical movement.
        initial = snapshot()
        if not initial:
            raise RuntimeError('Clock/odometry stale after follower discovery')
        if any(x['speed_m_s'] > a.stationary_speed for x in initial.values()):
            raise RuntimeError('One or more robots moving after follower discovery')
        if any(x['error_m'] > a.max_offset for x in initial.values()):
            raise RuntimeError('Offset exceeds limit after follower discovery')
        moved = [r for r in ROBOTS if initial[r]['error_m'] > a.position_tolerance]
        if a.execute:
            # Jazzy: use a normal rclpy service Client; AsyncParameterClient
            # does not provide wait_for_service() on this distribution.
            from rcl_interfaces.srv import GetParameters
            clients = {}
            for r in ROBOTS:
                service = f'/{r}/path_follower/get_parameters'
                client = node.create_client(GetParameters, service)
                clients[r] = client
                if not client.wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(f'{r}: parameter service unavailable: {service}')
                request = GetParameters.Request()
                request.names = ['waypoint_tolerance', 'hybrid_enabled']
                future = client.call_async(request)
                until = time.monotonic()+3
                while not future.done() and time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.1)
                if not future.done():
                    raise RuntimeError(f'{r}: parameter query timed out')
                response = future.result()
                if response is None or len(response.values) != 2:
                    raise RuntimeError(f'{r}: malformed parameter response')
                vals = response.values
                from rcl_interfaces.msg import ParameterType
                if (vals[0].type != ParameterType.PARAMETER_DOUBLE or
                        vals[1].type != ParameterType.PARAMETER_BOOL):
                    raise RuntimeError(f'{r}: wrong parameter types for tolerance/hybrid_enabled')
                tol = float(vals[0].double_value)
                enabled = bool(vals[1].bool_value)
                if not math.isfinite(tol) or tol <= 0:
                    raise RuntimeError(f'{r}: invalid waypoint tolerance {tol}')
                if tol > a.position_tolerance or not enabled:
                    raise RuntimeError(f'{r}: follower must be launched with waypoint_tolerance <= {a.position_tolerance} and hybrid_enabled=true (got {tol}, {enabled})')
            if not moved:
                state='ALREADY_ALIGNED'
            else:
                if any(pubs[r].get_subscription_count()<1 for r in moved):
                    raise RuntimeError('planned_path subscriber missing')
                for r in moved:
                    send_path(r, [initial[r]['xy'], targets[r]])
                stop_time = time.monotonic()+a.timeout
                settled = None
                recoveries = 0
                while time.monotonic()<stop_time:
                    rclpy.spin_once(node,timeout_sec=.08)
                    final, invalid_reason = snapshot_detailed()
                    if final is None:
                        # A route must be cancelled immediately on loss of trusted
                        # telemetry. Never let the existing follower keep its goal
                        # while the supervisor waits for messages to recover.
                        cancel_all()
                        settled = None
                        event = {'reason': invalid_reason, 'action': 'STOP_AND_WAIT',
                                 'wall_time_s': time.time(), 'recovery_index': recoveries+1}
                        recovery_events.append(event)
                        if invalid_reason.startswith(('FRAME_MISMATCH', 'INVALID_ODOMETRY')):
                            raise RuntimeError(f'Non-recoverable telemetry fault: {invalid_reason}')
                        recoveries += 1
                        if recoveries > a.max_recoveries:
                            raise RuntimeError(f'Recovery count exceeded ({a.max_recoveries}); last={invalid_reason}')
                        until = min(stop_time, time.monotonic()+a.recovery_seconds)
                        still_since = None
                        fresh = None
                        last_reason = invalid_reason
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=.08)
                            candidate, last_reason = snapshot_detailed()
                            if candidate is None:
                                still_since = None
                                continue
                            # Ensure the follower has decelerated to zero after
                            # cancellation before sending any new route.
                            if any(v['speed_m_s'] > a.stationary_speed for v in candidate.values()):
                                still_since = None
                                continue
                            if still_since is None:
                                still_since = time.monotonic()
                            elif time.monotonic()-still_since >= a.settle_seconds:
                                fresh = candidate
                                break
                        if fresh is None:
                            raise RuntimeError(f'Telemetry recovery failed after STOP: {last_reason}')
                        if any(v['error_m'] > a.max_offset+.15 for v in fresh.values()):
                            raise RuntimeError('Robot displaced too far to resume alignment')
                        # Reissue routes from measured positions; never resume
                        # the abandoned path without rechecking endpoints.
                        for r in moved:
                            if fresh[r]['error_m'] > a.position_tolerance:
                                send_path(r, [fresh[r]['xy'], targets[r]])
                        event['action'] = 'RECOVERED_RECHECKED_AND_RESENT'
                        event['remaining_errors_m'] = {r: fresh[r]['error_m'] for r in ROBOTS}
                        final = fresh
                        continue
                    if any(x['error_m'] > a.max_offset+.15 for x in final.values()):
                        raise RuntimeError('Robot deviated far from reference')
                    if all(x['error_m'] <= a.position_tolerance and x['speed_m_s'] <= a.stationary_speed for x in final.values()):
                        if settled is None:
                            settled=time.monotonic()
                        elif time.monotonic()-settled >= a.settle_seconds:
                            state='ALIGNED';break
                    else:
                        settled=None
                if state != 'ALIGNED':
                    raise RuntimeError('Timeout waiting for alignment')
        else:
            state='DRY_RUN_READY' if moved else 'ALREADY_ALIGNED'
    except Exception as exc:
        problems.append(str(exc));state='REJECT'
    finally:
        cancel_all()
        for _ in range(4):
            rclpy.spin_once(node,timeout_sec=.08)
        final = snapshot() or final
        if a.execute and state in ('ALIGNED','ALREADY_ALIGNED'):
            if not final or any(v['error_m'] > a.position_tolerance or v['speed_m_s'] > a.stationary_speed for v in final.values()):
                state='REJECT';problems.append('Alignment not stable after stop command')
        result={'schema':'swarm.d997.alignment.existing_path_follower.v1','status':state,
                'execution_requested':a.execute, 'main_trajectory_execution_authorized':False,
                'trajectory':str(a.trajectory.resolve()),'targets':targets,
                'initial':initial,'final':final,'alignment_robots':moved,'problems':problems,
                'graph_wait_s':round(graph_wait_s, 3), 'graph_publishers':graph_diagnostics,
                'recovery_events':recovery_events}
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps({'status':state,'alignment_robots':moved,'problems':problems,'report':str(a.output)},indent=2,ensure_ascii=False))
        node.destroy_node();rclpy.shutdown()
    return 0 if not problems else 2


if __name__ == '__main__':
    raise SystemExit(main())

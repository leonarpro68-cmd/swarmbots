#!/usr/bin/env python3
"""D9.6B: observe Gazebo /clock and evaluate three D9.4 references, READ ONLY.

No ROS publisher is instantiated. Strictly no cmd_vel or trajectory commands.
Runs one stage at a time, begins on first positive /clock observation.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path



class ClockGate:
    """Pure logic, independently testable without ROS installed."""
    def __init__(self, horizon, rewind_tolerance=1e-6):
        if not math.isfinite(horizon) or horizon <= 0:
            raise ValueError('Invalid horizon')
        self.horizon=float(horizon)
        self.start=None
        self.last=None
        self.rewind_tolerance=rewind_tolerance
        self.samples=0
        self.state='WAIT_CLOCK'

    def observe(self, stamp):
        if not math.isfinite(stamp) or stamp <= 0:
            return None
        if self.last is not None and stamp < self.last-self.rewind_tolerance:
            self.state='CLOCK_REWIND_ABORT'
            raise RuntimeError(f'/clock jumped backward: {self.last:.9f} -> {stamp:.9f}')
        if self.start is None:
            self.start=stamp
            self.state='RUNNING_READ_ONLY'
        self.last=max(stamp, self.last if self.last is not None else stamp)
        self.samples+=1
        elapsed=min(max(0.,self.last-self.start),self.horizon)
        if elapsed>=self.horizon:
            self.state='COMPLETE_READ_ONLY'
        return elapsed


def run_ros(run, stage, out, log_step=0.05, clock_timeout=15., pause_timeout=5., max_wall=180.):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
    except ImportError as exc:
        raise RuntimeError('ROS 2 Python packages not available. Source ROS 2 inside Docker.') from exc
    if not (0.005<=log_step<=1 and clock_timeout>0 and pause_timeout>0 and max_wall>0):
        raise ValueError('Invalid timing parameters')
    from d96a_readonly_follower import load_stage, sample
    manifest, trajectories=load_stage(run,stage)
    horizon=max(float(dt.sum()) for _,dt in trajectories.values())
    gate=ClockGate(horizon)
    out.mkdir(parents=True,exist_ok=True)
    csvpath=out/f'{stage}_D96B_CLOCK_READ_ONLY.csv'
    reportpath=out/f'{stage}_D96B_report.json'
    fields=['stage','clock_s','mission_t_s','robot','x_m','y_m','vx_m_s','vy_m_s','ax_m_s2','ay_m_s2','done']
    rclpy.init(args=None)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96b_clock_observer_read_only')
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.sub=self.create_subscription(Clock,'/clock',self.clock_callback,clock_qos)
            self.clock_messages=0
            self.last_written=-float('inf')
            self.last_wall_clock=time.monotonic()
            self.boot_wall=self.last_wall_clock
            self.stop_reason=None
            self.rows=0
            self.file=csvpath.open('w',newline='')
            self.writer=csv.DictWriter(self.file,fieldnames=fields)
            self.writer.writeheader()
            from rclpy.clock import Clock as RclClock
            from rclpy.clock_type import ClockType
            self.watchdog=self.create_timer(0.2,self.watchdog_callback,clock=RclClock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(f'D9.6B READ ONLY. Stage={stage} horizon={horizon:.3f}s. Awaiting /clock; NO COMMAND PUBLISHERS.')
        def clock_callback(self,msg):
            if self.stop_reason:return
            stamp=float(msg.clock.sec)+float(msg.clock.nanosec)*1e-9
            self.clock_messages+=1
            self.last_wall_clock=time.monotonic()
            try:t=gate.observe(stamp)
            except RuntimeError as exc:
                self.stop_reason=str(exc)
                return
            if t is None:return
            if t-self.last_written>=log_step or gate.state=='COMPLETE_READ_ONLY':
                states=sample(trajectories,t)
                for robot,st in states.items():
                    self.writer.writerow({'stage':stage,'clock_s':f'{stamp:.9f}','mission_t_s':f'{t:.9f}','robot':robot,**st})
                    self.rows+=1
                self.last_written=t
            if gate.state=='COMPLETE_READ_ONLY':
                self.stop_reason='COMPLETE_READ_ONLY'
        def watchdog_callback(self):
            if self.stop_reason:return
            now=time.monotonic()
            if now-self.boot_wall>max_wall:
                self.stop_reason='MAX_WALL_TIMEOUT'
            elif gate.start is None and now-self.boot_wall>clock_timeout:
                self.stop_reason='CLOCK_NOT_AVAILABLE'
            elif gate.start is not None and now-self.last_wall_clock>pause_timeout:
                self.stop_reason='CLOCK_STALLED_OR_GAZEBO_PAUSED'
    node=None
    try:
        node=Observer()
        while rclpy.ok() and not node.stop_reason:
            rclpy.spin_once(node,timeout_sec=0.2)
    finally:
        if node:
            node.file.flush();node.file.close()
            report={'stage':stage,'status':node.stop_reason or 'INTERRUPTED',
                    'ros_executable':False,'command_publishers_created':0,'commands_sent':0,
                    'clock_messages':node.clock_messages,'rows_written':node.rows,
                    'mission_elapsed_s':min(gate.horizon, max(0,(gate.last or 0)-(gate.start or 0))),
                    'planned_horizon_s':horizon,'clock_start_s':gate.start,
                    'clock_last_s':gate.last,'csv':str(csvpath),
                    'note':'Clock-first-sample origin is an observation anchor, NOT a synchronized robot start or execution certificate.'}
            reportpath.write_text(json.dumps(report,indent=2)+'\n')
            node.destroy_node()
        rclpy.shutdown()
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--stage',required=True,choices=['pickup1','delivery1','pickup2','delivery2'])
    ap.add_argument('--log-step',type=float,default=0.05)
    ap.add_argument('--clock-timeout',type=float,default=15.)
    ap.add_argument('--pause-timeout',type=float,default=5.)
    ap.add_argument('--max-wall',type=float,default=180.)
    a=ap.parse_args()
    result=run_ros(a.run,a.stage,a.run/'d96b_clock_readonly',a.log_step,a.clock_timeout,a.pause_timeout,a.max_wall)
    print(json.dumps(result,indent=2))
    if result['status']!='COMPLETE_READ_ONLY':raise SystemExit(2)

if __name__=='__main__':main()

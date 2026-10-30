#!/usr/bin/env python3
"""D9.9.15: grabador de odometría para comparación teoría vs realidad.

Registra odometría y reloj de simulación de summit0..2 en JSONL.
No publica comandos ni altera el movimiento.
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

ROBOTS = ("summit0", "summit1", "summit2")


class OdomRecorder(Node):
    def __init__(self, jsonl_path: Path, report_path: Path, min_dt: float = 0.0):
        super().__init__('d9915_odom_recorder')
        self.jsonl_path = jsonl_path
        self.report_path = report_path
        self.min_dt = float(min_dt)
        self._stop_evt = threading.Event()
        self.clock_s = None
        self.clock_count = 0
        self.odom_count = {r: 0 for r in ROBOTS}
        self.last_write_clock = {r: None for r in ROBOTS}
        self.first_clock_s = None
        self.last_clock_s = None
        self.fp = self.jsonl_path.open('w', encoding='utf-8')
        self.fp.write(json.dumps({
            'schema': 'swarm.d9916.odom_record.header.v1',
            'created_wall_s': time.time(),
            'robots': list(ROBOTS)
        }, ensure_ascii=False) + '\n')
        self.create_subscription(Clock, '/clock', self.on_clock, 50)
        for robot in ROBOTS:
            self.create_subscription(Odometry, f'/{robot}/odom', self._make_cb(robot), 50)
        self.flush_timer = self.create_timer(1.0, self.on_flush)

    def on_clock(self, msg: Clock):
        t = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.clock_s = t
        self.clock_count += 1
        if self.first_clock_s is None:
            self.first_clock_s = t
        self.last_clock_s = t

    def _make_cb(self, robot: str):
        def cb(msg: Odometry):
            c = self.clock_s
            if c is None:
                return
            prev = self.last_write_clock[robot]
            if prev is not None and (c - prev) < self.min_dt:
                return
            self.last_write_clock[robot] = c
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            tw = msg.twist.twist
            vx = float(tw.linear.x)
            vy = float(tw.linear.y)
            rec = {
                'clock_s': c,
                'robot': robot,
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'x_m': float(p.x),
                'y_m': float(p.y),
                'z_m': float(p.z),
                'qx': float(q.x), 'qy': float(q.y), 'qz': float(q.z), 'qw': float(q.w),
                'vx_m_s': vx,
                'vy_m_s': vy,
                'vz_m_s': float(tw.linear.z),
                'speed_m_s': (vx * vx + vy * vy) ** 0.5,
                'header_stamp_s': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            }
            self.fp.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self.odom_count[robot] += 1
        return cb

    def on_flush(self):
        try:
            self.fp.flush()
        except Exception:
            pass

    def stop(self):
        self._stop_evt.set()

    def write_report(self, status='STOPPED'):
        data = {
            'schema': 'swarm.d9916.odom_record.report.v1',
            'status': status,
            'jsonl': str(self.jsonl_path),
            'clock_count': self.clock_count,
            'first_clock_s': self.first_clock_s,
            'last_clock_s': self.last_clock_s,
            'odom_count': self.odom_count,
        }
        tmp = self.report_path.with_suffix(self.report_path.suffix + '.tmp')
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        os.replace(tmp, self.report_path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output-jsonl', default='scripts/resultados/d9915_odom.jsonl')
    ap.add_argument('--report', default='scripts/resultados/d9915_odom_report.json')
    ap.add_argument('--min-dt', type=float, default=0.0,
                    help='Submuestreo mínimo por robot en segundos de reloj de simulación')
    args = ap.parse_args()
    out = Path(args.output_jsonl).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    rep = Path(args.report).resolve()
    rep.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = OdomRecorder(out, rep, min_dt=args.min_dt)

    def _sigint(_signum, _frame):
        node.stop()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        while rclpy.ok() and not node._stop_evt.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.write_report('STOPPED')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'status': 'STOPPED', 'jsonl': str(out), 'report': str(rep)}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

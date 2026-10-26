#!/usr/bin/env python3
"""D.3: generar JSON timed_bezier.v1 para inspección offline (NO publica ROS)."""
import argparse
from swarm_behavior.swarm_planning.hybrid_v2.ros_protocol_adapter import build_commands,write_commands

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',required=True)
    p.add_argument('--stage',required=True,choices=['pickup1','delivery1','pickup2','delivery2'])
    p.add_argument('--controls')
    p.add_argument('--session',default='hybrid-v2-dryrun')
    p.add_argument('--start-time',type=float,required=True,help='Tiempo futuro en /clock; para esta prueba solo serialización')
    p.add_argument('--frames',nargs=3,required=True,metavar=('FRAME0','FRAME1','FRAME2'))
    p.add_argument('--output-dir',required=True)
    a=p.parse_args()
    commands=build_commands(a.report,a.stage,session_id=a.session,
            start_time_s=a.start_time,frame_ids=a.frames,controls_path=a.controls)
    directory=write_commands(commands,a.output_dir)
    print(f'DRY-RUN, SIN PUBLICAR ROS: {len(commands)} comandos en {directory}')
    return 0
if __name__=='__main__': raise SystemExit(main())

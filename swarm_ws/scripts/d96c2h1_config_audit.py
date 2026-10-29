#!/usr/bin/env python3
"""SwarmBots D9.6C.2H.1 — read-only odometry origin & generated-SDF audit.

Verifies source Xacro, launch routing, actual generated SDF per robot, and
D9.6C.2G empirical motion evidence. Does NOT create ROS/Gazebo publishers,
modify TF, authorize motion or certify a ROS TF tree.
"""
import argparse
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROBOTS=('summit0','summit1','summit2')
ODOM_PLUGIN='gz::sim::systems::OdometryPublisher'
MEC_PLUGIN='gz::sim::systems::MecanumDrive'
SCHEMA='swarm.d96c2h1.configuration.readonly.v1'


def sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            digest.update(chunk)
    return digest.hexdigest()


def tag_name(node):
    return node.tag.rsplit('}',1)[-1]


def plugins(xml_path):
    root=ET.parse(xml_path).getroot()
    return [node for node in root.iter() if tag_name(node)=='plugin']


def plugin_by_name(all_plugins,name):
    return [p for p in all_plugins if p.get('name','')==name]


def fields(plugin,names):
    return {name:next(((child.text or '').strip() for child in plugin if tag_name(child)==name),None)
            for name in names}


def verify_source(xacro_path,launch_path):
    p=plugins(xacro_path)
    odom=plugin_by_name(p,ODOM_PLUGIN)
    mecanum=plugin_by_name(p,MEC_PLUGIN)
    errors=[]
    if len(odom)!=1:errors.append('Xacro: se esperaba un único OdometryPublisher')
    if len(mecanum)!=1:errors.append('Xacro: se esperaba un único MecanumDrive')
    odom_config=fields(odom[0],('odom_frame','robot_base_frame','odom_topic','tf_topic','odom_publish_frequency')) if len(odom)==1 else {}
    mecanum_config=fields(mecanum[0],('odom_topic','tf_topic')) if len(mecanum)==1 else {}
    expected_odom={'odom_frame':'${ns}/odom','robot_base_frame':'${ns}/base_footprint',
                   'odom_topic':'/model/${ns}/odometry','tf_topic':'/model/${ns}/tf','odom_publish_frequency':'50'}
    expected_mec={'odom_topic':'/model/${ns}/mecanum_odom','tf_topic':'/model/${ns}/mecanum_tf'}
    for key,value in expected_odom.items():
        if odom_config.get(key)!=value:errors.append(f'Xacro OdometryPublisher.{key}: {odom_config.get(key)!r} != {value!r}')
    for key,value in expected_mec.items():
        if mecanum_config.get(key)!=value:errors.append(f'Xacro MecanumDrive.{key}: {mecanum_config.get(key)!r} != {value!r}')
    launch=launch_path.read_text(encoding='utf-8')
    required={
      'xacro utilizado para Summit omni': 'summit_xl_omni.urdf.xacro',
      'generación SDF': '_generate_sdf(ns_str, xacro_path',
      'puente odometría mundial': '/model/{ns_str}/odometry@nav_msgs/msg/Odometry',
      'puente odometría mecanum distinto': '/model/{ns_str}/mecanum_odom@nav_msgs/msg/Odometry',
      'remap de odometría mundial': '(f"/model/{ns_str}/odometry",     "odom")',
      'remap de odometría mecanum': '(f"/model/{ns_str}/mecanum_odom", "encoder_odom")',
    }
    for label,needle in required.items():
        if needle not in launch:errors.append(f'Launch: falta {label}')
    return {'passed':not errors,'errors':errors,'xacro_sha256':sha256(xacro_path),
            'launch_sha256':sha256(launch_path),
            'xacro_odom':odom_config,'xacro_mecanum':mecanum_config}


def verify_sdf(path,robot):
    all_plugins=plugins(path)
    odom=plugin_by_name(all_plugins,ODOM_PLUGIN)
    mec=plugin_by_name(all_plugins,MEC_PLUGIN)
    errors=[]
    if len(odom)!=1:errors.append('Debe haber un único OdometryPublisher')
    if len(mec)!=1:errors.append('Debe haber un único MecanumDrive')
    cfg=fields(odom[0],('odom_frame','robot_base_frame','odom_topic','tf_topic','odom_publish_frequency')) if len(odom)==1 else {}
    mc=fields(mec[0],('odom_topic','tf_topic')) if len(mec)==1 else {}
    wanted={
       'odom_frame':f'{robot}/odom',
       'robot_base_frame':f'{robot}/base_footprint',
       'odom_topic':f'/model/{robot}/odometry',
       'tf_topic':f'/model/{robot}/tf',
       'odom_publish_frequency':'50',
    }
    for k,v in wanted.items():
        if cfg.get(k)!=v:errors.append(f'OdometryPublisher.{k}: {cfg.get(k)!r} != {v!r}')
    for k,v in {'odom_topic':f'/model/{robot}/mecanum_odom',
                'tf_topic':f'/model/{robot}/mecanum_tf'}.items():
        if mc.get(k)!=v:errors.append(f'MecanumDrive.{k}: {mc.get(k)!r} != {v!r}')
    if cfg.get('odom_topic') == mc.get('odom_topic'):
        errors.append('PELIGRO: colisión entre odometrías de física y mecanum')
    return {'passed':not errors,'errors':errors,'sdf_sha256':sha256(path),
            'odom_plugin':cfg,'mecanum_plugin':mc,'file':str(path)}


def verify_dynamic_report(path):
    d=json.loads(path.read_text(encoding='utf-8'))
    errors=[]
    if d.get('status')!='DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN':
        errors.append('D9.6C.2G no declaró consistencia dinámica')
    if not d.get('sim_clock_progressing') or d.get('gazebo_truncated_samples',1)!=0:
        errors.append('Datos de reloj/samples insuficientes')
    if d.get('capture_warnings'):
        errors.append('Comparación dinámica tiene advertencias de captura')
    result={}
    for name in ROBOTS:
        row=d.get('robots',{}).get(name,{})
        good=(row.get('status')=='DYNAMIC_CONSISTENT'
              and row.get('near_identity_observed') is True
              and row.get('sufficient_motion') is True
              and row.get('odom_frames_expected') is True
              and row.get('matched_pairs',0)>=100
              and row.get('max_xy_delta_m',math.inf)<=.075
              and row.get('max_yaw_delta_deg',math.inf)<=4.0
              and row.get('max_timestamp_bracket_s',math.inf)<=.040001)
        result[name]={'passed':good,'pairs':row.get('matched_pairs'),
                      'max_xy_delta_m':row.get('max_xy_delta_m'),
                      'max_yaw_delta_deg':row.get('max_yaw_delta_deg')}
        if not good:errors.append(f'{name}: evidencia dinámica insuficiente')
    return {'passed':not errors,'errors':errors,'robots':result,'sha256':sha256(path),
            'note':'D9.6C.2G anterior usó límite de 40k muestras /clock; 2H.1 lo corrige, pero NO recertifica esa captura.'}


def run(args):
    report={'schema':SCHEMA,'status':'REJECT_CONFIGURATION_OR_EVIDENCE',
            'source_configuration':{},'generated_sdf':{},'dynamic_observation':{},
            'errors':[],'coordinate_frame_equivalence_proven':False,
            'tf_tree_certified':False,'execution_authorized':False,
            'ros_executable':False,'publishers_created':0,'commands_sent':0,
            'policy':'Solo lectura: aun si pasa, no autoriza publicar velocidades ni TF.'}
    try:
        report['source_configuration']=verify_source(args.source_xacro,args.launch)
        if not report['source_configuration']['passed']:
            report['errors']+=report['source_configuration']['errors']
    except (OSError,ET.ParseError,ValueError) as e:
        report['errors'].append('Fuentes: '+repr(e))
    for robot in ROBOTS:
        f=args.sdf_dir/f'{robot}.sdf'
        try:
            r=verify_sdf(f,robot)
            report['generated_sdf'][robot]=r
            if not r['passed']:report['errors'] += [robot+': '+error for error in r['errors']]
        except (OSError,ET.ParseError,ValueError) as e:
            report['generated_sdf'][robot]={'passed':False,'file':str(f),'error':repr(e)}
            report['errors'].append(robot+': SDF ausente o inválido: '+repr(e))
    try:
        report['dynamic_observation']=verify_dynamic_report(args.dynamic_report)
        if not report['dynamic_observation']['passed']:
            report['errors']+=report['dynamic_observation']['errors']
    except (OSError,ValueError,TypeError,KeyError) as e:
        report['errors'].append('D9.6C.2G: '+repr(e))
    if not report['errors']:
        report['status']='PASS_CONFIG_AND_DYNAMIC_OBSERVATION_NOT_TF_CERTIFIED'
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-xacro',type=Path,default=Path('src/summit_xl_description/robots/summit_xl_omni.urdf.xacro'))
    p.add_argument('--launch',type=Path,default=Path('src/swarm_worlds/launch/sim_summit.launch.py'))
    p.add_argument('--sdf-dir',type=Path,default=Path('/tmp/swarm_summit'))
    p.add_argument('--dynamic-report',type=Path,default=Path('scripts/resultados/d9_live/d96c2g_dynamic_frames.json'))
    p.add_argument('--output',type=Path,default=Path('scripts/resultados/d9_live/d96c2h1_configuration_report.json'))
    args=p.parse_args(argv)
    report=run(args)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status'].startswith('PASS_') else 2


if __name__=='__main__':sys.exit(main())

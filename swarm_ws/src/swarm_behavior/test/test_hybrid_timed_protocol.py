"""Pruebas puras del protocolo que usará ROS 2 (no requiere rclpy)."""
import json
import math

import pytest

from swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol import (
    TimedCommand, TimedProtocolError, build_status, decode_command,
    decode_status, derivative_bounds, evaluate_piece, reference,
)

P0 = ((0.,0.),(0.,0.),(0.,0.),(1.,0.),(1.,0.),(1.,0.))
WAIT = ((1.,0.),)*6


def command(pieces=(P0, WAIT), dt=10.):
    return TimedCommand('start','sesion-1','summit0',frame_id='odom',
                        start_time_s=20.,dt_s=dt,control_points=pieces,
                        speed_limit_m_s=.6,acceleration_limit_m_s2=.5,
                        max_tracking_error_m=.15,goal_tolerance_m=.1)


def test_roundtrip_y_espera():
    x=decode_command(command().to_json())
    assert x.duration_s==20.
    assert x.control_points[1]==WAIT
    assert x.frame_id=='odom'
    assert x.session_id=='sesion-1'


def test_reference_inicio_mitad_wait_final():
    c=decode_command(command().to_json())
    p,v,phase=reference(c,19.)
    assert p==(0.,0.) and v==(0.,0.) and phase=='before'
    p,v,phase=reference(c,25.)
    assert p==pytest.approx((.5,0.)) and phase=='active'
    assert v[0]>0.
    p,v,phase=reference(c,35.)
    assert p==pytest.approx((1.,0.)) and v==(0.,0.) and phase=='active'
    p,v,phase=reference(c,45.)
    assert p==(1.,0.) and v==(0.,0.) and phase=='after'


def test_rapidez_y_aceleracion_cotas_analiticas():
    vmax,amax=derivative_bounds((P0,),10.)
    assert vmax==pytest.approx(.5)
    assert amax==pytest.approx(.2)


def test_eval_extremos():
    assert evaluate_piece(P0,0,10.)==((0.,0.),(0.,0.))
    assert evaluate_piece(P0,1,10.)==((1.,0.),(0.,0.))


def test_rechaza_velocidad_excesiva():
    with pytest.raises(TimedProtocolError,match='dinámicos'):
        decode_command(command((P0,),dt=2.).to_json())


def test_rechaza_discontinuidad_en_posicion():
    bad=json.loads(command().to_json())
    bad['control_points_m'][1][0]=[1.1,0.]
    with pytest.raises(TimedProtocolError,match='discontinua'):
        decode_command(json.dumps(bad))


def test_rechaza_espera_con_salto_velocidad():
    bad=json.loads(command().to_json())
    bad['control_points_m'][1][1]=[1.1,0.]
    with pytest.raises(TimedProtocolError,match='Velocidad'):
        decode_command(json.dumps(bad))


def test_rechaza_frame_malformado():
    bad=json.loads(command().to_json());bad['frame_id']=''
    with pytest.raises(TimedProtocolError):decode_command(json.dumps(bad))


def test_rechaza_numero_nan():
    bad=json.loads(command().to_json());bad['control_points_m'][0][0][0]=float('nan')
    with pytest.raises(TimedProtocolError):decode_command(json.dumps(bad))


def test_rechaza_error_seguimiento_demasiado_alto():
    bad=json.loads(command().to_json());bad['max_tracking_error_m']=.9
    with pytest.raises(TimedProtocolError,match='Límites'):
        decode_command(json.dumps(bad))


def test_rechaza_many_segments():
    bad=json.loads(command().to_json());bad['control_points_m']=[bad['control_points_m'][1]]*201
    with pytest.raises(TimedProtocolError,match='Número de piezas'):
        decode_command(json.dumps(bad))


def test_cancelacion_y_estado():
    c=decode_command(TimedCommand('cancel','sesion-1','summit0').to_json())
    assert c.action=='cancel'
    assert decode_status(build_status('summit0','sesion-1','armed'))==('summit0','sesion-1','armed')
    with pytest.raises(TimedProtocolError): decode_status('{"state":"armed"}')


def test_frame_con_namespace_ros():
    raw=json.loads(command().to_json())
    raw['frame_id']='rb_0/odom'
    assert decode_command(json.dumps(raw)).frame_id=='rb_0/odom'

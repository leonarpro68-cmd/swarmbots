"""Fase 5: protocolo seguro de trayectorias Bézier temporizadas (sin ROS).

El mensaje String de ROS usa este JSON validado. El reloj de referencia es el
simulado de ROS (/clock). La posición está expresada en el mismo sistema
numérico global que los ejecutores CBS actuales y el marco de odometría local.

No constituye garantía contra fallos de actuadores o comunicaciones.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re

SCHEMA = 'swarmbots.timed_bezier.v1'
SCHEMA_V2 = 'swarmbots.timed_bezier.v2'
MAX_SEGMENTS = 200
MAX_SEGMENTS_V2 = 512
MAX_PAYLOAD_BYTES = 300_000


class TimedProtocolError(ValueError):
    pass


def _positive_float(x, name):
    if isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) or x <= 0:
        raise TimedProtocolError(f'{name}: número positivo y finito requerido')
    return float(x)


def _string(x, name):
    if not isinstance(x, str) or not x or len(x) > 128 or not re.fullmatch(r'[\w.\-]+', x):
        raise TimedProtocolError(f'{name}: identificador inválido')
    return x


def _frame(x):
    if not isinstance(x,str) or not x or len(x)>128 or not re.fullmatch(r'[A-Za-z0-9_./\-]+',x):
        raise TimedProtocolError('frame_id inválido')
    return x


@dataclass(frozen=True)
class TimedCommand:
    action: str
    session_id: str
    robot: str
    frame_id: str | None = None
    start_time_s: float | None = None
    dt_s: float | None = None
    control_points: tuple = ()
    speed_limit_m_s: float = .6
    acceleration_limit_m_s2: float = .5
    max_tracking_error_m: float = .15
    goal_tolerance_m: float = .12
    goal_settle_s: float = .5
    finish_grace_s: float = 4.
    segment_durations_s: tuple = ()

    @property
    def duration_s(self):
        return sum(self.segment_durations_s) if self.segment_durations_s else (len(self.control_points) * self.dt_s if self.dt_s is not None else 0.)

    def to_dict(self):
        out = dict(schema=SCHEMA_V2 if self.segment_durations_s else SCHEMA, action=self.action, session_id=self.session_id, robot=self.robot)
        if self.action == 'start':
            out.update(frame_id=self.frame_id, start_time_s=self.start_time_s,
                       dt_s=self.dt_s, control_points_m=self.control_points,
                       speed_limit_m_s=self.speed_limit_m_s,
                       acceleration_limit_m_s2=self.acceleration_limit_m_s2,
                       max_tracking_error_m=self.max_tracking_error_m,
                       goal_tolerance_m=self.goal_tolerance_m,
                       goal_settle_s=self.goal_settle_s,
                       finish_grace_s=self.finish_grace_s)
        if self.action == 'start' and self.segment_durations_s:
            out.pop('dt_s', None)
            out['segment_durations_s'] = self.segment_durations_s
        return out

    def to_json(self):
        return json.dumps(self.to_dict(), separators=(',', ':'), allow_nan=False)


def decode_command(raw: str) -> TimedCommand:
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > MAX_PAYLOAD_BYTES:
        raise TimedProtocolError('Mensaje excesivamente grande o inválido')
    try:
        data = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nan/inf')))
    except (ValueError, TypeError) as e:
        raise TimedProtocolError('JSON inválido') from e
    if not isinstance(data, dict) or data.get('schema') != SCHEMA:
        raise TimedProtocolError('Esquema de protocolo desconocido')
    action = data.get('action')
    if action not in ('start', 'cancel'):
        raise TimedProtocolError('Acción desconocida')
    session = _string(data.get('session_id'), 'session_id')
    robot = _string(data.get('robot'), 'robot')
    if action == 'cancel':
        return TimedCommand(action, session, robot)
    frame = _frame(data.get('frame_id'))
    start = _positive_float(data.get('start_time_s'), 'start_time_s')
    dt = _positive_float(data.get('dt_s'), 'dt_s')
    if dt < .02 or dt > 100.:
        raise TimedProtocolError('dt_s fuera de límite')
    points = data.get('control_points_m')
    if not isinstance(points, list) or not (1 <= len(points) <= MAX_SEGMENTS):
        raise TimedProtocolError('Número de piezas Bézier incorrecto')
    pieces=[]
    for piece in points:
        if not isinstance(piece, (tuple,list)) or len(piece)!=6:
            raise TimedProtocolError('Cada pieza debe contener 6 puntos')
        row=[]
        for point in piece:
            if not isinstance(point,(tuple,list)) or len(point)!=2 or any(
                isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v)
                or abs(v)>1000 for v in point
            ):
                raise TimedProtocolError('Punto de control inválido')
            row.append((float(point[0]),float(point[1])))
        pieces.append(tuple(row))
    # Invariantes geométricas mínimas; la certificación integral viene de Fase 4.
    for a,b in zip(pieces, pieces[1:]):
        for d in range(2):
            if abs(a[-1][d]-b[0][d])>1e-5:
                raise TimedProtocolError('Trayectoria discontinua')
            if abs((a[-1][d]-a[-2][d])-(b[1][d]-b[0][d]))>1e-5:
                raise TimedProtocolError('Velocidad discontinua')
            if abs((a[-1][d]-2*a[-2][d]+a[-3][d])-
                   (b[2][d]-2*b[1][d]+b[0][d]))>1e-5:
                raise TimedProtocolError('Aceleración discontinua')
    speed = _positive_float(data.get('speed_limit_m_s'), 'speed_limit_m_s')
    accel = _positive_float(data.get('acceleration_limit_m_s2'), 'acceleration_limit_m_s2')
    error = _positive_float(data.get('max_tracking_error_m'), 'max_tracking_error_m')
    goal = _positive_float(data.get('goal_tolerance_m'), 'goal_tolerance_m')
    settle = _positive_float(data.get('goal_settle_s'), 'goal_settle_s')
    grace = _positive_float(data.get('finish_grace_s'), 'finish_grace_s')
    if speed > 1.20 or accel > 1.00 or not .03 <= error <= .25 or goal > error or grace > 30:
        raise TimedProtocolError('Límites exceden los valores autorizados para demostración')
    vmax,amax = derivative_bounds(tuple(pieces), dt)
    if vmax > speed+1e-6 or amax > accel+1e-6:
        raise TimedProtocolError(f'Límites dinámicos de la referencia excedidos: {vmax:.3f}, {amax:.3f}')
    return TimedCommand(action,session,robot,frame,start,dt,tuple(pieces),
                        speed,accel,error,goal,settle,grace)


def derivative_bounds(pieces, dt):
    vm = am = 0.
    for piece in pieces:
        first=[((piece[j+1][0]-piece[j][0])*5/dt,
                (piece[j+1][1]-piece[j][1])*5/dt) for j in range(5)]
        second=[((first[j+1][0]-first[j][0])*4/dt,
                 (first[j+1][1]-first[j][1])*4/dt) for j in range(4)]
        vm=max(vm,*(math.hypot(*p) for p in first))
        am=max(am,*(math.hypot(*p) for p in second))
    return vm,am


def evaluate_piece(piece, u, dt):
    """Posición y velocidad cartesiana mundial (m, m/s)."""
    u=max(0.,min(1.,float(u)))
    def bernstein(pts):
        n=len(pts)-1
        x=y=0.
        for j,(px,py) in enumerate(pts):
            w=math.comb(n,j)*((1-u)**(n-j))*(u**j)
            x+=w*px; y+=w*py
        return x,y
    p=bernstein(piece)
    deriv=[((piece[j+1][0]-piece[j][0])*5/dt,
            (piece[j+1][1]-piece[j][1])*5/dt) for j in range(5)]
    return p,bernstein(deriv)


def reference(cmd: TimedCommand, now_s: float):
    """Devuelve posición, feedforward, fase (before/active/after)."""
    if cmd.segment_durations_s:
        import bisect
        t = now_s - cmd.start_time_s
        if t < 0:
            return cmd.control_points[0][0], (0.,0.), 'before'
        if t >= cmd.duration_s:
            return cmd.control_points[-1][-1], (0.,0.), 'after'
        cumulative = [0.]
        for dt_piece in cmd.segment_durations_s:
            cumulative.append(cumulative[-1] + dt_piece)
        k = min(bisect.bisect_right(cumulative, t)-1, len(cmd.control_points)-1)
        dt_piece = cmd.segment_durations_s[k]
        return (*evaluate_piece(cmd.control_points[k],(t-cumulative[k])/dt_piece,dt_piece),'active')
    t=now_s-cmd.start_time_s
    if t<0:
        return cmd.control_points[0][0],(0.,0.),'before'
    if t>=cmd.duration_s:
        return cmd.control_points[-1][-1],(0.,0.),'after'
    k=min(int(t/cmd.dt_s),len(cmd.control_points)-1)
    p,v=evaluate_piece(cmd.control_points[k],(t-k*cmd.dt_s)/cmd.dt_s,cmd.dt_s)
    return p,v,'active'


def build_status(robot: str, session: str, state: str, detail: str='') -> str:
    return json.dumps(dict(schema=SCHEMA,robot=robot,session_id=session,
                           state=state,detail=detail[:250]),ensure_ascii=False)


def decode_status(raw: str):
    try:d=json.loads(raw)
    except (TypeError, ValueError) as e: raise TimedProtocolError('Estado no JSON') from e
    if not isinstance(d,dict) or d.get('schema')!=SCHEMA or d.get('state') not in (
        'armed','running','done','failed','cancelled'):
        raise TimedProtocolError('Estado incorrecto')
    return _string(d.get('robot'),'robot'),_string(d.get('session_id'),'session_id'),d['state']

# D9.9 extension: V1 decoder is preserved without changing its validation or semantics.
_decode_command_v1 = decode_command

def decode_command(raw: str) -> TimedCommand:
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > MAX_PAYLOAD_BYTES:
        raise TimedProtocolError('Mensaje excesivamente grande o inválido')
    try:
        obj = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nan/inf')))
    except (ValueError, TypeError) as exc:
        raise TimedProtocolError('JSON inválido') from exc
    if not isinstance(obj, dict):
        raise TimedProtocolError('Comando no es un objeto JSON')
    if obj.get('schema') != SCHEMA_V2:
        return _decode_command_v1(raw)
    if obj.get('action') == 'cancel':
        return TimedCommand('cancel',_string(obj.get('session_id'),'session_id'),_string(obj.get('robot'),'robot'))
    if obj.get('action') != 'start':
        raise TimedProtocolError('Acción desconocida')
    session = _string(obj.get('session_id'),'session_id')
    robot = _string(obj.get('robot'),'robot')
    frame = _frame(obj.get('frame_id'))
    start = _positive_float(obj.get('start_time_s'),'start_time_s')
    pts = obj.get('control_points_m')
    ds = obj.get('segment_durations_s')
    if not isinstance(pts,list) or not 1<=len(pts)<=MAX_SEGMENTS_V2:
        raise TimedProtocolError('Número de piezas incorrecto')
    if not isinstance(ds,list) or len(ds)!=len(pts):
        raise TimedProtocolError('Duraciones deben coincidir con el número de piezas')
    durations = tuple(_positive_float(d,'segment_duration') for d in ds)
    if any(d<.02 or d>100 for d in durations):
        raise TimedProtocolError('Duración fuera de límite')
    pieces=[]
    for piece in pts:
        if not isinstance(piece,list) or len(piece)!=6: raise TimedProtocolError('Pieza inválida')
        row=[]
        for point in piece:
            if (not isinstance(point,(list,tuple)) or len(point)!=2 or
                any(isinstance(v,bool) or not isinstance(v,(int,float)) or
                    not math.isfinite(v) or abs(v)>1000 for v in point)):
                raise TimedProtocolError('Punto inválido')
            row.append((float(point[0]),float(point[1])))
        pieces.append(tuple(row))
    speed = _positive_float(obj.get('speed_limit_m_s'),'speed_limit_m_s')
    accel = _positive_float(obj.get('acceleration_limit_m_s2'),'acceleration_limit_m_s2')
    error = _positive_float(obj.get('max_tracking_error_m'),'max_tracking_error_m')
    goal = _positive_float(obj.get('goal_tolerance_m'),'goal_tolerance_m')
    settle = _positive_float(obj.get('goal_settle_s'),'goal_settle_s')
    grace = _positive_float(obj.get('finish_grace_s'),'finish_grace_s')
    if speed>1.2 or accel>1. or not .03<=error<=.25 or goal>error or grace>30:
        raise TimedProtocolError('Límites no autorizados')
    for j,(a,b) in enumerate(zip(pieces,pieces[1:])):
        da,db = durations[j],durations[j+1]
        for axis in (0,1):
            if abs(a[-1][axis]-b[0][axis])>1e-5:
                raise TimedProtocolError('Discontinuidad C0')
            if abs(5*(a[-1][axis]-a[-2][axis])/da - 5*(b[1][axis]-b[0][axis])/db)>1e-3:
                raise TimedProtocolError('Discontinuidad C1 temporal')
            aa=20*(a[-1][axis]-2*a[-2][axis]+a[-3][axis])/(da*da)
            bb=20*(b[2][axis]-2*b[1][axis]+b[0][axis])/(db*db)
            if abs(aa-bb)>1e-2: raise TimedProtocolError('Discontinuidad C2 temporal')
    for piece,d in zip(pieces,durations):
        vmax,amax=derivative_bounds((piece,),d)
        if vmax>speed+1e-6 or amax>accel+1e-6:
            raise TimedProtocolError('Límites dinámicos excedidos')
    return TimedCommand('start',session,robot,frame,start,None,tuple(pieces),speed,accel,error,goal,settle,grace,durations)

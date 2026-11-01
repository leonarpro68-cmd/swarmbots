"""Reloj virtual sucesivo D9.9.31, sin ROS.

    tau: tiempo de la trayectoria CBS/Bezier original.
    t(tau) = integral_0^tau 1/q(u) du, q=d(tau)/dt>0.

El MISMO q(tau) se transmite a los 3 seguidores. Para los intervalos sin
rampa se invierte analiticamente y para las rampas mediante biseccion y
cuadratura Gauss-Legendre. No se aproxima con reescalado de tramos.
"""
from __future__ import annotations
import bisect
import math

MODEL='common_collective_successive_v1'
# Gauss-Legendre 24 puntos; error vs D9929/32 validado en tests.
_X=(-0.995187219997021311,-0.974728555971309474,-0.938274552002732798,-0.886415527004401071,-0.820001985973902947,-0.740124191578554358,-0.648093651936975546,-0.545421471388839563,-0.433793507626045127,-0.315042679696163397,-0.191118867473616311,-0.06405689286260563,0.06405689286260563,0.191118867473616311,0.315042679696163397,0.433793507626045127,0.545421471388839563,0.648093651936975546,0.740124191578554358,0.820001985973902947,0.886415527004401071,0.938274552002732798,0.974728555971309474,0.995187219997021311)
_W=(0.0123412297999886903,0.0285313886289335593,0.0442774388174194122,0.0592985849154363601,0.0733464814110801611,0.086190161531953205,0.097618652104113926,0.107444270115965565,0.115505668053725516,0.121670472927803294,0.125837456346828247,0.127938195346752021,0.127938195346752021,0.125837456346828247,0.121670472927803294,0.115505668053725516,0.107444270115965565,0.097618652104113926,0.086190161531953205,0.0733464814110801611,0.0592985849154363601,0.0442774388174194122,0.0285313886289335593,0.0123412297999886903)


def smoothstep(s):
    return s*s*s*(10+s*(-15+6*s))


def smoothstep_prime(s):
    return 30*s*s*(1-s)*(1-s)


class SuccessiveClock:
    def __init__(self, baseline, ramps, horizon, claimed_wall=None):
        self.baseline=float(baseline)
        self.horizon=float(horizon)
        if not 1.0 <= self.baseline <= 2.0 or not math.isfinite(self.horizon) or self.horizon <= 0:
            raise ValueError('RELOJ_V3_BASE_HORIZON_INVALID')
        self.ramps=[]
        prev_end=0.0
        previous_scale=self.baseline
        for r in ramps:
            if not isinstance(r,dict): raise ValueError('Rampa no es objeto')
            a=float(r['start_tau']); b=float(r['end_tau']); q0=float(r['from_scale']); q1=float(r['to_scale'])
            if (not all(map(math.isfinite,(a,b,q0,q1))) or a < prev_end-1e-9 or
                b <= a+1e-8 or b > self.horizon+1e-8 or
                abs(q0-previous_scale)>1e-6 or q1 < q0-1e-9 or q1 > 2.+1e-8):
                raise ValueError('RAMPA_V3_INVALIDA')
            self.ramps.append({'start_tau':a,'end_tau':b,'from_scale':q0,'to_scale':q1})
            prev_end=b;previous_scale=q1
        # Construir intervalos constantes y rampas para inversion rapida.
        self.spans=[]
        pos=0.0;q=self.baseline; wall=0.0
        for r in self.ramps:
            a,b=r['start_tau'],r['end_tau']
            if a>pos:
                delta=(a-pos)/q
                self.spans.append((pos,a,wall,wall+delta,'constant',q,None))
                wall+=delta;pos=a
            delta=self._quad(a,b,r)
            self.spans.append((a,b,wall,wall+delta,'ramp',None,r))
            wall+=delta;pos=b;q=r['to_scale']
        if self.horizon>pos:
            delta=(self.horizon-pos)/q
            self.spans.append((pos,self.horizon,wall,wall+delta,'constant',q,None))
            wall+=delta
        self.duration_s=wall
        if claimed_wall is not None and abs(self.duration_s-float(claimed_wall))>0.0001:
            raise ValueError(f'DURACION_RELOJ_V3_DISTINTA {self.duration_s:.8f} vs {claimed_wall}')
        self._wall_ends=[x[3] for x in self.spans]
        self._tau_ends=[x[1] for x in self.spans]

    def _q_ramp(self,tau,r):
        u=max(0.,min(1.,(tau-r['start_tau'])/(r['end_tau']-r['start_tau'])))
        return r['from_scale']+(r['to_scale']-r['from_scale'])*smoothstep(u)

    def _quad(self,left,right,r):
        if right<=left:return 0.
        mid=(left+right)/2;half=(right-left)/2
        return half*sum(w/self._q_ramp(mid+half*x,r) for x,w in zip(_X,_W))

    def scale(self,tau):
        q=self.baseline
        for r in self.ramps:
            if tau<r['start_tau']:return q
            if tau<r['end_tau']:return self._q_ramp(tau,r)
            q=r['to_scale']
        return q

    def max_qprime(self,left,right):
        peak=0.
        for r in self.ramps:
            a=max(left,r['start_tau']);b=min(right,r['end_tau'])
            if b<=a:continue
            width=r['end_tau']-r['start_tau']
            x=(a-r['start_tau'])/width;y=(b-r['start_tau'])/width
            pts=[x,y]+([.5] if x<=.5<=y else [])
            peak=max(peak,(r['to_scale']-r['from_scale'])/width*max(smoothstep_prime(p) for p in pts))
        return peak

    def wall(self,tau):
        if tau<=0:return 0.
        if tau>=self.horizon:return self.duration_s
        i=bisect.bisect_right(self._tau_ends,tau)
        a,b,wa,wb,kind,q,r=self.spans[i]
        return wa+((tau-a)/q if kind=='constant' else self._quad(a,tau,r))

    def inverse(self,elapsed):
        if elapsed<=0:return 0.
        if elapsed>=self.duration_s:return self.horizon
        i=bisect.bisect_right(self._wall_ends,elapsed)
        a,b,wa,wb,kind,q,r=self.spans[i]
        if kind=='constant':return a+(elapsed-wa)*q
        lo=a;hi=b
        for _ in range(28):
            mid=(lo+hi)/2
            if self._quad(a,mid,r)<elapsed-wa:lo=mid
            else:hi=mid
        return (lo+hi)/2


def clock_from_payload(payload):
    if not isinstance(payload,dict) or payload.get('model')!=MODEL:
        raise ValueError('MODELO_RELOJ_V3_INVALIDO')
    return SuccessiveClock(payload['baseline_scale'],payload['ramps'],payload['duration_original_s'],payload['duration_successive_s'])


def command_envelopes(control_points, durations, clock):
    """Cotas Bernstein + regla de la cadena, para UN robot V3."""
    if len(control_points)!=len(durations) or not durations:
        raise ValueError('TRAMOS_V3_NO_COINCIDEN')
    pos=0.;peak_v=peak_a=0.
    for piece,dt in zip(control_points,durations):
        dt=float(dt)
        if dt<=0:raise ValueError('DURACION_V3_INVALIDA')
        v=max(math.hypot(5*(piece[j+1][0]-piece[j][0])/dt,
                         5*(piece[j+1][1]-piece[j][1])/dt) for j in range(5))
        acc=max(math.hypot(20*(piece[j+2][0]-2*piece[j+1][0]+piece[j][0])/(dt*dt),
                           20*(piece[j+2][1]-2*piece[j+1][1]+piece[j][1])/(dt*dt)) for j in range(4))
        q=clock.scale(pos+dt)
        prime=clock.max_qprime(pos,pos+dt)
        peak_v=max(peak_v,v*q)
        peak_a=max(peak_a,acc*q*q+v*q*prime)
        pos+=dt
    if abs(pos-clock.horizon)>0.0001:raise ValueError('HORIZONTE_V3_NO_COINCIDE')
    return peak_v,peak_a

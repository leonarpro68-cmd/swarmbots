"""D9.2 harmonic phase model and conservative verifier, adapted from d92_smooth_retime.py. Offline only."""
import numpy as np
PI=np.pi


def phase(x, a, b):
    return x+a*np.sin(2*PI*x)/(2*PI)+b*np.sin(4*PI*x)/(4*PI)


def phase_d1(x,a,b):
    return 1+a*np.cos(2*PI*x)+b*np.cos(4*PI*x)


def phase_d2(x,a,b):
    return -2*PI*a*np.sin(2*PI*x)-4*PI*b*np.sin(4*PI*x)


def phase_d3_bound(a,b):
    return (2*PI)**2*abs(a)+(4*PI)**2*abs(b)


def values_for_robot(c,n,T,a,b,t):
    """Vectorized Cartesian p,v,acc for samples in the motion interval."""
    times=np.asarray(t,dtype=float)
    x=np.clip(times/T,0,1)
    u=np.clip(phase(x,a,b),0,1)
    q=u*n
    indices=np.minimum(np.floor(q).astype(int),n-1)
    w=np.clip(q-indices,0,1)
    controls=c[indices]
    one=1-w
    # Vectorized de Casteljau evaluates position and derivative control polygons.
    def evalpoly(poly):
        work=poly.copy()
        for level in range(poly.shape[1]-1,0,-1):
            work=one[:,None,None]*work[:,:level,:]+w[:,None,None]*work[:,1:level+1,:]
        return work[:,0,:]
    pos=evalpoly(controls)
    d1=evalpoly(5*(controls[:,1:,:]-controls[:,:-1,:]))
    d2=evalpoly(20*(controls[:,2:,:]-2*controls[:,1:-1,:]+controls[:,:-2,:]))
    qd=n*phase_d1(x,a,b)/T
    qdd=n*phase_d2(x,a,b)/T**2
    vel=d1*qd[:,None]
    acc=d2*(qd**2)[:,None]+d1*qdd[:,None]
    vel=np.where((times<T)[:,None],vel,0)
    acc=np.where((times<T)[:,None],acc,0)
    return pos,vel,acc


def verify(cs,ns,Ts,coeff,vmax,amax,sep,step):
    if any(n<1 for n in ns):return False,{'reason':'stationary_trajectory_not_supported'}
    if any(T<=0 or 1-abs(a)-abs(b)<=0.05 for T,(a,b) in zip(Ts,coeff)):
        return False,{'reason':'invalid_monotonicity'}
    per=[]; certified_v=[];certified_a=[]
    positions=[]
    # Include exact segment joints in time samples by inversion: avoids silently
    # crossing a piece boundary in a bound based on one side's jerk.
    for c,n,T,(a,b) in zip(cs,ns,Ts,coeff):
        for j in range(1,n):
            if np.linalg.norm(c[j-1,-1]-c[j,0])>1e-7 or np.linalg.norm(5*(c[j-1,-1]-c[j-1,-2])-5*(c[j,1]-c[j,0]))>1e-7:
                return False,{'reason':'non_C1_join'}
            if np.linalg.norm(20*(c[j-1,-1]-2*c[j-1,-2]+c[j-1,-3])-20*(c[j,2]-2*c[j,1]+c[j,0]))>1e-6:
                return False,{'reason':'non_C2_join'}
        base=np.linspace(0,T,max(2,int(np.ceil(T/step))+1))
        goals=np.arange(n+1,dtype=float)/n
        lo=np.zeros(n+1); hi=np.ones(n+1)
        for _ in range(52):
            mid=(lo+hi)/2
            mask=phase(mid,a,b)<goals
            lo=np.where(mask,mid,lo);hi=np.where(mask,hi,mid)
        knot_x=(lo+hi)/2
        times=np.unique(np.r_[base,T*knot_x])
        p,v,acc=values_for_robot(c,n,T,a,b,times)
        if np.linalg.norm(c[0,1]-c[0,0])>1e-7 or np.linalg.norm(c[n-1,-1]-c[n-1,-2])>1e-7:
            return False,{"reason":"nonzero_endpoint_velocity"}
        s=np.linalg.norm(v,axis=1); ac=np.linalg.norm(acc,axis=1)
        # Local third-derivative bound for each active piece via control polygons.
        d1=5*np.diff(c[:n],axis=1)
        d2=20*np.diff(c[:n],n=2,axis=1)
        d3=60*np.diff(c[:n],n=3,axis=1)
        M1=np.linalg.norm(d1,axis=2).max(axis=1)
        M2=np.linalg.norm(d2,axis=2).max(axis=1)
        M3=np.linalg.norm(d3,axis=2).max(axis=1)
        q1=n*(1+abs(a)+abs(b))/T
        q2=n*(2*PI*abs(a)+4*PI*abs(b))/T**2
        q3=n*phase_d3_bound(a,b)/T**3
        jerk=M3*q1**3+3*M2*q1*q2+M1*q3
        # Samples of v/a do not by themselves certify the intervals.
        # For each interval, choose segment by phase of midpoint; joints inserted.
        mids=(times[:-1]+times[1:])/2
        u_m=phase(mids/T,a,b)
        ids=np.minimum((u_m*n).astype(int),n-1)
        hs=np.diff(times)
        # nearest-endpoint Lipschitz extensions; bound v by measured acc bound
        # obtained from acceleration + jerk covering intervals.
        a_edges=np.maximum(ac[:-1],ac[1:])+jerk[ids]*hs/2
        a_cert=float(np.max(a_edges))
        v_edges=np.maximum(s[:-1],s[1:])+a_edges*hs/2
        v_cert=float(np.max(v_edges))
        if v_cert>vmax+1e-9 or a_cert>amax+1e-9:
            return False,{'reason':'dynamic_conservative_bound','speed_upper':v_cert,
                          'accel_upper':a_cert,'robot':len(per)}
        per.append({'duration_s':T,'speed_upper_m_s':v_cert,'accel_upper_m_s2':a_cert,
                    'max_sampled_speed_m_s':float(np.max(s)),'max_sampled_accel_m_s2':float(np.max(ac))})
        certified_v.append(v_cert);certified_a.append(a_cert)
    horizon=max(Ts)
    grid=np.linspace(0,horizon,max(2,int(np.ceil(horizon/step))+1))
    pos=[values_for_robot(c,n,T,a,b,grid)[0] for c,n,T,(a,b) in zip(cs,ns,Ts,coeff)]
    min_sampled=float('inf');min_lower=float('inf')
    h=float(grid[1]-grid[0]);pairs=[]
    for i in range(3):
        for j in range(i+1,3):
            ds=np.linalg.norm(pos[i]-pos[j],axis=1)
            dd=float(np.min(ds));low=dd-(certified_v[i]+certified_v[j])*h/2
            pairs.append({'robots':[i,j],'min_sampled_m':dd,'lower_m':low})
            min_sampled=min(dd,min_sampled);min_lower=min(low,min_lower)
            if low+1e-9<sep:
                return False,{'reason':'separation','min_lower_m':min_lower,'pair':[i,j]}
    return True,{'makespan_s':horizon,'robots':per,'min_sampled_m':min_sampled,
                 'certified_lower_m':min_lower,'pairs':pairs,'sample_step_s':h}


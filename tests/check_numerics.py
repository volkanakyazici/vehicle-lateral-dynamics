"""Independent numerical preflight; does NOT run MATLAB or Simulink.

Creates explicitly labelled Python preview CSVs to exercise the analysis code.
The deliverable's primary simulation workflow is matlab/run_project.m.
"""
from pathlib import Path
import json
import sys
import numpy as np
import scipy
from scipy.linalg import expm, solve_discrete_are

ROOT = Path(__file__).resolve().parents[1]
P = json.loads((ROOT / 'config.json').read_text())


def matrices(u, dt):
    m, iz = P['mass_kg'], P['yaw_inertia_kgm2']
    a, b = P['front_distance_m'], P['rear_distance_m']
    cf, cr = P['front_axle_stiffness_Nprad'], P['rear_axle_stiffness_Nprad']
    A = np.array([[-(cf+cr)/(m*u), (b*cr-a*cf)/(m*u)-u, 0, 0],
                  [(b*cr-a*cf)/(iz*u), -(a*a*cf+b*b*cr)/(iz*u), 0, 0],
                  [0, 1, 0, 0], [1, 0, u, 0]], dtype=float)
    B = np.array([[cf/m], [a*cf/iz], [0], [0]])
    aug = np.zeros((5, 5)); aug[:4, :4] = A; aug[:4, 4:] = B
    zoh = expm(aug * dt)
    return A, B, zoh[:4, :4], zoh[:4, 4:]


def gains(ad, bd):
    q = np.diag(1 / np.array(P['lqr_state_scales'])**2)
    rr = 1 / P['lqr_input_scale_rad']**2
    ric = solve_discrete_are(ad, bd, q, np.array([[rr]]))
    k = np.linalg.solve(rr + bd.T @ ric @ bd, bd.T @ ric @ ad)
    return k, ric, q, rr


def iterative_gain(ad, bd, q, rr):
    """Exercise the same Riccati iteration used in the MATLAB source."""
    ric = q.copy()
    for _ in range(20000):
        k = np.linalg.solve(rr + bd.T @ ric @ bd, bd.T @ ric @ ad)
        nxt = q + ad.T @ ric @ ad - ad.T @ ric @ bd @ k
        nxt = (nxt + nxt.T) / 2
        converged = np.linalg.norm(nxt-ric) <= 1e-12*max(1, np.linalg.norm(nxt))
        ric = nxt
        if converged:
            return np.linalg.solve(rr + bd.T @ ric @ bd, bd.T @ ric @ ad)
    raise AssertionError('Riccati iteration did not converge')


def case(kind, u, dt=None):
    dt = P['sample_time_s'] if dt is None else dt
    duration = 10.0 if kind in ['step', 'sine', 'zero'] else P['path_end_m']/u
    t = np.arange(round(duration/dt)+1) * dt
    xr = np.zeros((len(t), 4)); ff = np.zeros(len(t)); road = np.zeros(len(t))
    slope = np.zeros(len(t)); curvature = np.zeros(len(t)); open_delta = np.zeros(len(t))
    amp = np.deg2rad(P['steering_amplitude_deg'])
    if kind == 'step': open_delta[(t >= 1) & (t < 5)] = amp
    if kind == 'sine':
        active = (t >= 1) & (t < 7)
        open_delta[active] = amp*np.sin(2*np.pi*.5*(t[active]-1))
    if kind not in ['step', 'sine', 'zero']:
        length = P['path_length_m']; shift = P['path_shift_m']
        q = np.clip((u*t-P['path_start_m'])/length, 0, 1)
        road = shift*(10*q**3 - 15*q**4 + 6*q**5)
        slope = shift/length*(30*q**2 - 60*q**3 + 30*q**4)
        curvature = shift/length**2*(60*q - 180*q**2 + 120*q**3)
        a, b = P['front_distance_m'], P['rear_distance_m']; length_axles = a+b
        m = P['mass_kg']; cf=P['front_axle_stiffness_Nprad']; cr=P['rear_axle_stiffness_Nprad']
        ku = m/length_axles*(b/cf-a/cr)
        beta_ref = (b-m*a*u*u/(length_axles*cr))*curvature
        xr = np.column_stack((u*beta_ref, u*curvature, slope-beta_ref, road))
        ff = (length_axles+ku*u*u)*curvature
    limit = np.deg2rad(P['steering_limit_deg'])
    rate = np.deg2rad(P['steering_rate_limit_degps'])
    if kind == 'limited': limit = np.deg2rad(.15); rate=np.deg2rad(3)
    x0 = np.array([0., 0., 0., .5 if kind == 'offset' else 0.])
    return dict(kind=kind, u=u, dt=dt, t=t, xr=xr, ff=ff, road=road,
                slope=slope, curvature=curvature, open_delta=open_delta,
                tracking=kind in ['track','offset','limited'], limit=limit, rate=rate, x0=x0)


def simulate(c):
    _, _, ad, bd = matrices(c['u'], c['dt']); k, _, _, _ = gains(ad, bd)
    x = np.zeros((len(c['t']), 4)); x[0] = c['x0']
    raw=np.zeros(len(x)); delta=np.zeros(len(x)); prev=0.
    for i in range(len(x)):
        raw[i] = c['ff'][i] + (k @ (c['xr'][i]-x[i])).item() if c['tracking'] else c['open_delta'][i]
        angle = np.clip(raw[i], -c['limit'], c['limit'])
        delta[i] = prev + np.clip(angle-prev, -c['rate']*c['dt'], c['rate']*c['dt'])
        if i+1 < len(x): x[i+1] = ad @ x[i] + bd[:,0]*delta[i]
        prev = delta[i]
    return x, raw, delta


def quantities(c, x, raw, delta):
    u=c['u'];a=P['front_distance_m'];b=P['rear_distance_m']
    af=delta-(x[:,0]+a*x[:,1])/u; ar=-(x[:,0]-b*x[:,1])/u
    fyf=P['front_axle_stiffness_Nprad']*af; fyr=P['rear_axle_stiffness_Nprad']*ar
    columns = ['time_s','X_m','Y_m','Y_ref_m','vy_mps','yaw_rate_radps','psi_rad',
               'road_heading_rad','body_heading_ref_rad','beta_rad','delta_request_rad',
               'delta_rad','alpha_f_rad','alpha_r_rad','Fyf_N','Fyr_N','ay_mps2',
               'ey_m','epsi_road_rad','curvature_1pm']
    values=np.column_stack((c['t'],u*c['t'],x[:,3],c['road'],x[:,0],x[:,1],x[:,2],
                            c['slope'],c['xr'][:,2],x[:,0]/u,raw,delta,af,ar,fyf,fyr,
                            (fyf+fyr)/P['mass_kg'],x[:,3]-c['road'],x[:,2]-c['slope'],c['curvature']))
    return columns, values


def run_checks():
    checks=[]
    def ok(name, condition):
        if not condition: raise AssertionError(name)
        checks.append({'name':name,'status':'PASS'})
    for u in P['speeds_mps']:
        A,B,ad,bd=matrices(u,P['sample_time_s']);k,ric,q,rr=gains(ad,bd)
        ok(f'U={u:g}: lateral poles stable', np.max(np.real(np.linalg.eigvals(A[:2,:2])))<0)
        ok(f'U={u:g}: closed-loop poles inside unit circle', np.max(abs(np.linalg.eigvals(ad-bd@k)))<1)
        ok(f'U={u:g}: Riccati residual',np.linalg.norm(ric-(q+ad.T@ric@ad-ad.T@ric@bd@k))/np.linalg.norm(ric)<1e-10)
        ok(f'U={u:g}: iterative gain agrees with SciPy',np.max(abs(iterative_gain(ad,bd,q,rr)-k))<1e-8)
        # Compare the matrix derivative with independently calculated forces.
        x=np.array([.1,.02,.03,.4]);delta=.004;a=P['front_distance_m'];b=P['rear_distance_m']
        ff=P['front_axle_stiffness_Nprad']*(delta-(x[0]+a*x[1])/u)
        fr=P['rear_axle_stiffness_Nprad']*(-(x[0]-b*x[1])/u)
        direct=np.array([(ff+fr)/P['mass_kg']-u*x[1],(a*ff-b*fr)/P['yaw_inertia_kgm2'],x[1],x[0]+u*x[2]])
        ok(f'U={u:g}: force balance and matrix derivative', np.max(abs(A@x+B[:,0]*delta-direct))<1e-12)
        ku=P['mass_kg']/(a+b)*(b/P['front_axle_stiffness_Nprad']-a/P['rear_axle_stiffness_Nprad'])
        steady=-np.linalg.solve(A[:2,:2],B[:2,0]*delta)
        ok(f'U={u:g}: analytic steady yaw gain', abs(steady[1]-u*delta/(a+b+ku*u*u))<1e-12)
        zero=case('zero',u);xz,_,_=simulate(zero)
        ok(f'U={u:g}: zero input remains zero',np.max(abs(xz))==0)
        c=case('step',u);xs,raw,ds=simulate(c)
        cm=case('step',u);cm['open_delta']*=-1;xm,_,_=simulate(cm)
        ok(f'U={u:g}: steering sign symmetry',np.max(abs(xs+xm))<1e-12)
        step_ss=-np.linalg.solve(A[:2,:2],B[:2,0]*np.deg2rad(P['steering_amplitude_deg']))
        ok(f'U={u:g}: step reaches lateral equilibrium',np.max(abs(xs[np.argmin(abs(c['t']-4.99)),:2]-step_ss))<1e-5)
    all_cases=[(s,u) for u in P['speeds_mps'] for s in ['step','uncontrolled','track']]+[('sine',20.),('offset',20.),('limited',20.)]
    out=ROOT/'results'/'preview';out.mkdir(parents=True,exist_ok=True)
    for kind,u in all_cases:
        c=case(kind,u);x,raw,d=simulate(c);columns,data=quantities(c,x,raw,d)
        name=f'{kind}_{int(u)}'
        ok(f'{name}: finite outputs',np.isfinite(data).all())
        ok(f'{name}: angle and rate limits',np.max(abs(d))<=c['limit']+1e-12 and np.max(abs(np.diff(np.r_[0,d])))<=c['rate']*c['dt']+1e-12)
        if kind in ['track','offset']:
            ok(f'{name}: path RMSE below 0.15 m',np.sqrt(np.mean(data[:-1,17]**2))<.15)
            ok(f'{name}: final path error below 0.02 m',abs(data[-1,17])<.02)
        if kind=='limited':ok('limited case activates angle constraint',np.any(abs(raw)>c['limit']+1e-9))
        np.savetxt(out/f'{name}.csv',data,delimiter=',',header=','.join(columns),comments='',fmt='%.12g')
        meta={'source':'PYTHON_NUMERICAL_PREVIEW_NOT_SIMULINK','case':kind,'speed_mps':u,
              'sample_time_s':c['dt'],'steering_limit_rad':c['limit'],'steering_rate_limit_radps':c['rate'],
              'config':P,'gain':gains(matrices(u,c['dt'])[2],matrices(u,c['dt'])[3])[0].tolist()}
        (out/f'{name}.json').write_text(json.dumps(meta,indent=2)+'\n')
        error = f'RMSE {np.sqrt(np.mean(data[:-1,17]**2)):.5f} m; ' if kind in ['track','offset','limited','uncontrolled'] else ''
        print(f"{name}: {error}max slip {np.rad2deg(np.max(abs(data[:,12:14]))):.3f} deg; max |ay| {max(abs(data[:,16]))/P['gravity_mps2']:.3f} g")
    # Fine-rate sampled controller check; this is not a Simulink execution.
    for u in P['speeds_mps']:
        c=case('track',u);xf,_,_=simulate(case('track',u,c['dt']/2));xc,_,_=simulate(c)
        ok(f'U={u:g}: sample-time refinement < 0.02 m',np.max(abs(xc[:,3]-xf[::2,3]))<.02)
    report={'status':'PASS','environment':'Python '+sys.version.split()[0], 'matlab_simulink':'NOT_RUN',
            'numpy_version':np.__version__,'scipy_version':scipy.__version__,
            'checks':checks,'count':len(checks)}
    (out/'preflight_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(f'PASS: {len(checks)} numerical checks. MATLAB/Simulink: NOT RUN.')


if __name__=='__main__':
    report_path=ROOT/'results'/'preview'/'preflight_report.json'
    report_path.parent.mkdir(parents=True,exist_ok=True)
    report_path.write_text(json.dumps({'status':'RUNNING','matlab_simulink':'NOT_RUN'})+'\n')
    try:
        run_checks()
    except Exception as error:
        report_path.write_text(json.dumps({'status':'FAIL','matlab_simulink':'NOT_RUN','error':str(error)},indent=2)+'\n')
        raise

"""Deterministic numerical evidence; these are calculated data, not physics measurements."""
import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import robots,Serial,Delta,Stewart,plan,quintic,pose_values,pose_error,pose_matrix
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'reports';OUT.mkdir(exist_ok=True)
rng=np.random.default_rng(20261006);summaries=[]
for name,r in robots().items():
    if isinstance(r,Stewart):
        goals=[r.home,r.ik(r.home_pose+np.array([.035,0,.025,.04,0,0])),r.ik(r.home_pose+np.array([0,.035,0,0,.04,0])),r.home]
    else:
        a=r.home.copy();b=a.copy();c=a.copy();b[0]+=.22;c[1]+=.20
        if name=='SCARA':b[2]=.09;c[2]=.025
        goals=[a,b,c,a]
    segments=plan(r,r.home,goals,4);rows=[];clock=0.;seed=r.fk(r.home);max_closure=0
    for a,b,d,_ in segments:
        for u in np.linspace(0,1,101):
            q=a+(b-a)*quintic(u);v=(b-a)*30*u*u*(1-u)**2/d;acc=(b-a)*(60*u-180*u*u+120*u**3)/d**2
            t=r.fk(q,seed) if isinstance(r,Stewart) else r.fk(q);seed=t
            if isinstance(r,Delta):closure=np.max(np.abs(np.linalg.norm(t[:3,3]+r.offset-r.elbows(q),axis=1)-r.lower));max_closure=max(max_closure,float(closure))
            if isinstance(r,Stewart):max_closure=max(max_closure,float(np.max(np.abs(r.lengths(pose_values(t))-q))))
            rows.append([clock+u*d,*q,*v,*acc,*pose_values(t)])
        clock+=d
    data=np.array(rows);n=len(r.home)
    with (OUT/f'{name}_calculated_trajectory.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time_s']+[f'q{i+1}_{u}' for i,u in enumerate(r.units)]+[f'v{i+1}_{u}_s' for i,u in enumerate(r.units)]+[f'a{i+1}_{u}_s2' for i,u in enumerate(r.units)]+['x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad']);w.writerows(rows)
    points=[]
    for _ in range(1200):
        try:
            if isinstance(r,Stewart):
                p=np.r_[rng.uniform([-.22,-.22,.30],[.22,.22,.65]),0,0,0];r.ik(p);points.append(p[:3])
            else:points.append(r.fk(rng.uniform(r.limits[:,0],r.limits[:,1]))[:3,3])
        except ValueError:pass
    points=np.array(points);np.savetxt(OUT/f'{name}_workspace.csv',points,delimiter=',',header='x_m,y_m,z_m',comments='')
    fig=plt.figure(figsize=(9,6));ax=fig.add_subplot(111,projection='3d');ax.scatter(*points.T,s=2,alpha=.18,color='#168aad',label='Position samples')
    for a,b in r.segments(r.home):ax.plot(*np.array([a,b]).T,color='#183a69',lw=3)
    path=data[:,-6:-3];ax.plot(*path.T,color='#b5176b',lw=2,label='Calculated tool path');ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title=name+' — calculated workspace and trajectory');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(OUT/f'{name}_workspace.png',dpi=140);plt.close(fig)
    t=r.fk(goals[1]);qi=r.ik(pose_values(t),r.home);err=pose_error(r.fk(qi),t)
    summaries.append({'robot':name,'duration_s':clock,'workspace_points':len(points),'workspace_attempts':1200,'ik_position_residual_m':float(np.linalg.norm(err[:3])),'ik_orientation_residual_rad':float(np.linalg.norm(err[3:])),'max_sampled_loop_residual_m':max_closure if not isinstance(r,Serial) else None,'max_velocity_limit_ratio':float(np.max(abs(data[:,1+n:1+2*n])/r.vmax)),'max_acceleration_limit_ratio':float(np.max(abs(data[:,1+2*n:1+3*n])/r.amax)),'target_pose':pose_values(t).tolist(),'solved_actuators':qi.tolist()})
    print(name, 'calculated evidence exported')
(OUT/'six_robot_numerical_results.json').write_text(json.dumps(summaries,indent=2))

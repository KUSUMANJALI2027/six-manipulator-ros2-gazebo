"""Analyze recorded simulator CSVs; no automatic accuracy pass/fail threshold."""
import argparse, csv, hashlib, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def analyze(path, output):
    with path.open() as f:
        reader=csv.DictReader(f); names=reader.fieldnames; rows=list(reader)
    if not rows: raise ValueError(f'{path.name}: empty recording')
    data={k:np.array([float(r[k]) for r in rows]) for k in names}
    if not all(np.isfinite(v).all() for v in data.values()):
        raise ValueError(f'{path.name}: contains non-finite values')
    t=data['simulation_time_s']; dt=np.diff(t)
    if np.any(dt<0): raise ValueError(f'{path.name}: time reset; analyze sessions separately')
    commands=[k for k in names if k.startswith('cmd_')]
    if not commands: raise ValueError('No command columns')
    robot=path.name.split('_physics_feedback')[0]
    fig,axes=plt.subplots(len(commands),2,figsize=(10,2.05*len(commands)),squeeze=False)
    stats=[]; tail=t>=t[-1]-5
    for i,k in enumerate(commands):
        unit=k.rsplit('_',1)[1]; factor=1000 if unit=='m' else 180/np.pi; label='mm' if unit=='m' else 'deg'
        q=data[k]*factor; m=data[k.replace('cmd_','measured_',1)]*factor; error=m-q
        span=float(np.ptp(q)); row={'robot':robot,'joint':i+1,'unit':label,
            'command_span':span,'command_varied':span>1e-5,
            'final_abs_error':float(abs(error[-1])),
            'sample_rmse':float(np.sqrt(np.mean(error**2))),
            'peak_abs_error':float(np.max(abs(error))),
            'last_5s_abs_error_max':float(np.max(abs(error[tail])))}
        stats.append(row)
        ax=axes[i,0];ax.plot(t,q,label='Command',lw=1.4);ax.plot(t,m,label='Gazebo feedback',lw=.9);ax.set_ylabel(f'J{i+1} ({label})');ax.grid(alpha=.25)
        ax=axes[i,1];ax.plot(t,error,lw=.9,color='#b84b31');ax.set_ylabel(f'Error ({label})');ax.grid(alpha=.25)
    
    for ax in axes.flat:ax.ticklabel_format(axis='y',style='plain',useOffset=False)
    axes[0,0].legend(fontsize=8);axes[0,0].set_title('Command and measured joint state');axes[0,1].set_title('Measured minus command')
    for ax in axes[-1]:ax.set_xlabel('Simulation time (s)')
    fig.suptitle(robot+' — recorded Gazebo trial',y=1.0);fig.tight_layout();fig.savefig(output/(robot+'_tracking.png'),dpi=150);plt.close(fig)
    return {'source':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'samples':len(rows),
        'time_start_s':float(t[0]),'time_end_s':float(t[-1]),'duplicate_timestamps':int(np.sum(dt==0)),
        'last_window_duration_s':float(t[-1]-t[tail][0]),'joints':stats}

def main():
    p=argparse.ArgumentParser();p.add_argument('directory',nargs='?',type=Path,default=Path(__file__).parent/'evidence');p.add_argument('--output',type=Path,default=Path(__file__).parent/'reports');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);files=sorted(a.directory.glob('*_physics_feedback_*.csv'))
    if not files:raise SystemExit('No feedback CSVs found')
    names=[f.name.split('_physics_feedback')[0] for f in files]
    if len(names)!=len(set(names)):raise SystemExit('Use one recording per robot in the input folder to avoid overwriting plots.')
    result=[analyze(f,a.output) for f in files]
    (a.output/'feedback_analysis.json').write_text(json.dumps(result,indent=2))
    with (a.output/'joint_error_summary.csv').open('w',newline='') as f:
        rows=[j for r in result for j in r['joints']];w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print('Analyzed',len(result),'recordings. Sample RMSE is sample-weighted, including duplicate timestamps.')
    print('Command variation indicates test coverage only. No hardware accuracy or universal stability claim.')
if __name__=='__main__':main()

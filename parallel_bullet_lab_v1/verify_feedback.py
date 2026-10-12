"""Engineering investigation gates, not certified industrial tolerances."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from kinematics_reference import Delta,Stewart,pose_values,pose_error

def verify(path,robot):
 r=Delta() if robot=='Delta' else Stewart()
 a=np.genfromtxt(path,delimiter=',',names=True);a=np.atleast_1d(a)
 if len(a)<2:raise ValueError('Insufficient samples')
 for n in a.dtype.names:
  if not np.isfinite(a[n]).all():raise ValueError('Nonfinite data in '+n)
 n=len(r.home);unit='rad' if robot=='Delta' else 'm';q=np.column_stack([a[f'measured_{i+1}_{unit}'] for i in range(n)]);cmd=np.column_stack([a[f'cmd_{i+1}_{unit}'] for i in range(n)])
 times=a['simulation_time_s'];error=q-cmd
 if np.any(np.diff(times)<=0):raise ValueError('Time must increase')
 span=np.ptp(cmd,axis=0);tail=times>=times[-1]-5
 p=np.column_stack([a[k] for k in ['tool_x','tool_y','tool_z']]);quat=np.column_stack([a[k] for k in ['qx','qy','qz','qw']]);max_pos=0.;max_rot=0.;seed=r.fk(r.home)
 for idx in range(0,len(a),5):
  predicted=r.fk(q[idx],seed) if robot=='Stewart' else r.fk(q[idx]);seed=predicted
  actual=np.eye(4);actual[:3,3]=p[idx];actual[:3,:3]=Rotation.from_quat(quat[idx]).as_matrix();e=pose_error(actual,predicted);max_pos=max(max_pos,float(np.linalg.norm(e[:3])));max_rot=max(max_rot,float(np.linalg.norm(e[3:])))
 result={'robot':robot,'file':path.name,'samples':len(a),'end_time_s':float(times[-1]),'command_span':span.tolist(),'units':unit,'max_abs_tracking_error':np.max(abs(error),axis=0).tolist(),'last_5s_max_abs_error':np.max(abs(error[tail]),axis=0).tolist(),'max_anchor_or_slider_gap_m':float(np.max(a['max_anchor_gap_m'])),'sampled_pose_vs_measured_actuator_fk_position_m':max_pos,'sampled_pose_vs_measured_actuator_fk_rotation_rad':max_rot}
 result['gates']={'measured_actuators_within_model_limits':bool(np.all(q>=r.limits[:,0]-1e-4) and np.all(q<=r.limits[:,1]+1e-4)),'full_trial_recorded':bool(times[-1]>=(5+12*n+10)-.02),'each_actuator_command_varied':bool(np.all(span>(.039 if robot=='Delta' else .0039))),'gap_below_1mm':result['max_anchor_or_slider_gap_m']<.001,'fk_position_below_1mm':max_pos<.001,'fk_orientation_below_0_01rad':max_rot<.01,'tail_tracking_below_0_01rad_or_1mm':bool(np.max(abs(error[tail]))<(.01 if robot=='Delta' else .001))}
 result['investigation_gates_passed']=all(result['gates'].values());return result

def main():
 p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--suffix',default='_core_feedback.csv');args=p.parse_args()
 results=[verify(args.directory/(robot+args.suffix),robot) for robot in ['Delta','Stewart']]
 out=args.directory/'verification.json';out.write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
 if not all(x['investigation_gates_passed'] for x in results):raise SystemExit('One or more investigation gates failed; send results ZIP.')
if __name__=='__main__':main()

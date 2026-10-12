from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
for robot,n,unit in [('Delta',3,'rad'),('Stewart',6,'m')]:
 a=np.genfromtxt(root/'evidence'/'parallel'/f'{robot}_gazebo_feedback.csv',delimiter=',',names=True)
 fig,axes=plt.subplots(3,2,figsize=(10,6),squeeze=False)
 for i,ax in enumerate(axes.flat):
  if i>=n:ax.axis('off');continue
  factor=180/np.pi if unit=='rad' else 1000;label='deg' if unit=='rad' else 'mm'
  ax.plot(a['simulation_time_s'],a[f'cmd_{i+1}_{unit}']*factor,label='Command',lw=1.4)
  ax.plot(a['simulation_time_s'],a[f'measured_{i+1}_{unit}']*factor,label='Measured',lw=.9)
  ax.set(xlabel='Simulation time (s)',ylabel=f'J{i+1} ({label})');ax.grid(alpha=.25);ax.ticklabel_format(axis='y',style='plain',useOffset=False)
 axes[0,0].legend(fontsize=8);fig.suptitle(robot+' — custom Bullet physics inside Gazebo');fig.tight_layout();fig.savefig(root/'reports'/f'{robot}_parallel_tracking.png',dpi=150);plt.close(fig)

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.family':'serif','font.size':8,'axes.labelsize':8,'legend.fontsize':7,'xtick.labelsize':7,'ytick.labelsize':7,'pdf.fonttype':42})
fig,axs=plt.subplots(2,2,figsize=(7.0,3.3),layout='constrained')
for ax,name in zip(axs.flat,['SCARA','Stanford','PUMA','SCORBOT']):
 a=np.genfromtxt(next((ROOT/'data').glob(name+'_physics*.csv')),delimiter=',',names=True)
 for j in [1,2]:
  ax.plot(a['simulation_time_s'],np.rad2deg(a[f'measured_{j}_rad']-a[f'cmd_{j}_rad']),lw=.65,label=f'J{j}')
 ax.set(title=name,xlabel='Simulation time (s)',ylabel='Tracking error (deg)');ax.grid(alpha=.25);ax.legend()
fig.savefig(ROOT/'figures/serial_errors.pdf');plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(7.0,4.4),layout='constrained')
for col,name in enumerate(['Delta','Stewart']):
 a=np.genfromtxt(ROOT/'data'/f'{name}_gazebo_feedback.csv',delimiter=',',names=True)
 t=a['simulation_time_s'];unit='rad' if name=='Delta' else 'm';n=3 if name=='Delta' else 6;factor=1 if n==3 else 1000
 for i in range(1,n+1):
  c=a[f'cmd_{i}_{unit}'];m=a[f'measured_{i}_{unit}'];color=f'C{i-1}'
  axs[0,col].plot(t,factor*(c-c[0]),'--',lw=.8,color=color)
  axs[0,col].plot(t,factor*(m-c[0]),lw=.65,color=color,label=f'A{i}')
  axs[1,col].plot(t,factor*(m-c),lw=.65,color=color)
 axs[0,col].set_title(name);axs[0,col].set_ylabel('Offset (rad)' if n==3 else 'Offset (mm)');axs[0,col].legend(ncol=3)
 axs[1,col].set_ylabel('Error (rad)' if n==3 else 'Error (mm)')
 axs[2,col].plot(t,a['max_anchor_gap_m']*1e6,lw=.7,color='black');axs[2,col].set(ylabel='Constraint gap (µm)',xlabel='Simulation time (s)')
 for row in range(3):axs[row,col].grid(alpha=.25)
fig.savefig(ROOT/'figures/parallel_tracking.pdf');plt.close(fig)

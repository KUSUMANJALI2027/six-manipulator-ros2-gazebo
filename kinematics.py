"""Six educational manipulators. SI units; angles radians. Not vendor CAD models."""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def dh(theta,d,a,alpha):
    c,s,C,S=np.cos(theta),np.sin(theta),np.cos(alpha),np.sin(alpha)
    return np.array([[c,-s*C,s*S,a*c],[s,c*C,-c*S,a*s],[0,S,C,d],[0,0,0,1.]])

def pose_matrix(p):
    t=np.eye(4);t[:3,3]=p[:3];t[:3,:3]=Rotation.from_euler('xyz',p[3:]).as_matrix();return t

def pose_values(t):
    return np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_euler('xyz')]

def pose_error(actual,target):
    return np.r_[actual[:3,3]-target[:3,3],Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]

class Serial:
    def __init__(self,name,rows,limits,home):
        self.name=name;self.rows=rows;self.limits=np.array(limits,float);self.home=np.array(home,float)
        self.units=['m' if row[0]=='P' else 'rad' for row in rows]
        self.vmax=np.array([.12 if u=='m' else .8 for u in self.units])
        self.amax=np.array([.25 if u=='m' else 1.6 for u in self.units])
    def frames(self,q):
        frames=[np.eye(4)]
        for value,(kind,theta,d,a,alpha,sign) in zip(q,self.rows):
            frames.append(frames[-1]@dh(theta+(sign*value if kind=='R' else 0),d+(sign*value if kind=='P' else 0),a,alpha))
        return frames
    def fk(self,q):return self.frames(q)[-1]
    def ik(self,p,seed=None):
        target=pose_matrix(p); seed=self.home if seed is None else np.array(seed)
        if not np.isfinite(target).all():raise ValueError('Target must be finite.')
        seeds=[seed,self.home]+[np.random.default_rng(i).uniform(self.limits[:,0],self.limits[:,1]) for i in range(6)]
        for guess in seeds:
            fit=least_squares(lambda q:pose_error(self.fk(q),target),np.clip(guess,self.limits[:,0]+1e-8,self.limits[:,1]-1e-8),bounds=self.limits.T,max_nfev=250,gtol=1e-10,ftol=1e-10,xtol=1e-10)
            if np.linalg.norm(pose_error(self.fk(fit.x),target))<1e-5:return fit.x
        raise ValueError('No IK solution found within limits. Target may be unreachable or solver may need another seed.')
    def segments(self,q):
        f=self.frames(q);return [(a[:3,3],b[:3,3]) for a,b in zip(f,f[1:])]
    def table(self,q):
        return [[str(i+1),k,f'{t+(v*sg if k=="R" else 0):.4f}',f'{d+(v*sg if k=="P" else 0):.4f}',f'{a:.4f}',f'{al:.4f}'] for i,(v,(k,t,d,a,al,sg)) in enumerate(zip(q,self.rows))]

class Delta:
    name='Delta';units=['rad']*3;home=np.array([.30,.30,.30]);limits=np.array([[-.35,1.15]]*3)
    vmax=np.array([.8]*3);amax=np.array([1.6]*3)
    rb=.28;rp=.075;upper=.20;lower=.48;top=.85
    def __init__(self):
        a=np.arange(3)*2*np.pi/3;self.rad=np.c_[np.cos(a),np.sin(a),np.zeros(3)]
        self.base=self.rb*self.rad+np.array([0,0,self.top]);self.offset=self.rp*self.rad
    def elbows(self,q):return self.base+self.upper*(np.cos(q)[:,None]*self.rad-np.sin(q)[:,None]*np.array([0,0,1]))
    def fk(self,q):
        centers=self.elbows(q)-self.offset
        # Intersection of three equal-radius spheres; choose lower assembly mode.
        ex=centers[1]-centers[0];d=np.linalg.norm(ex);ex/=d
        v=centers[2]-centers[0];i=np.dot(ex,v);ey=v-i*ex;j=np.linalg.norm(ey)
        if j<1e-9:raise ValueError('Delta singular geometry.')
        ey/=j;ez=np.cross(ex,ey);x=d/2;y=(i*i+j*j-2*i*x)/(2*j);z2=self.lower**2-x*x-y*y
        if z2<0:raise ValueError('Delta links cannot close at these actuator angles.')
        mid=centers[0]+x*ex+y*ey;opts=[mid+np.sqrt(z2)*ez,mid-np.sqrt(z2)*ez]
        p=min(opts,key=lambda p:p[2]);return pose_matrix(np.r_[p,0,0,0])
    def ik(self,p,seed=None):
        if np.linalg.norm(p[3:])>1e-7:raise ValueError('Delta controls XYZ only; set roll, pitch and yaw to zero.')
        p=np.array(p[:3]);out=[]
        for base,r,off in zip(self.base,self.rad,self.offset):
            v=p+off-base;A=np.dot(v,r);B=-v[2];C=(np.dot(v,v)+self.upper**2-self.lower**2)/(2*self.upper)
            radius=np.hypot(A,B)
            if radius<1e-9 or abs(C)>radius:raise ValueError('Delta target outside geometric reach.')
            phase=np.arctan2(B,A);delta=np.arccos(np.clip(C/radius,-1,1));opts=[]
            for s in (-1,1):
                for n in (-1,0,1):
                    q=phase+s*delta+2*np.pi*n
                    if self.limits[0,0]<=q<=self.limits[0,1]:opts.append(q)
            if not opts:raise ValueError('Delta actuator limit exceeded.')
            out.append(min(opts,key=lambda q:abs(q-(self.home[len(out)] if seed is None else seed[len(out)]))))
        out=np.array(out)
        if np.linalg.norm(self.fk(out)[:3,3]-p)>1e-5:raise ValueError('Target is on another Delta assembly branch.')
        return out
    def segments(self,q):
        tip=self.fk(q)[:3,3];e=self.elbows(q);p=tip+self.offset;out=[]
        for i in range(3):
            out.extend([(self.base[i],e[i]),(p[i],p[(i+1)%3]),(self.base[i],self.base[(i+1)%3])])
            tangent=np.cross([0,0,1],self.rad[i])*.018
            out.extend([(e[i]+tangent,p[i]+tangent),(e[i]-tangent,p[i]-tangent)])
        return out
    def table(self,q):return [[str(i+1),'R',f'{q[i]:.4f}',f'{self.upper:.4f}',f'{self.lower:.4f}',f'{self.rb}/{self.rp}'] for i in range(3)]

class Stewart:
    name='Stewart';units=['m']*6;limits=np.array([[.30,.75]]*6);vmax=np.array([.12]*6);amax=np.array([.25]*6)
    def __init__(self):
        b=np.deg2rad([10,110,130,230,250,350]);p=np.deg2rad([50,70,170,190,290,310])
        self.base=np.c_[.32*np.cos(b),.32*np.sin(b),np.zeros(6)]
        self.platform=np.c_[.20*np.cos(p),.20*np.sin(p),np.zeros(6)]
        self.home_pose=np.array([0,0,.48,0,0,0]);self.home=self.ik(self.home_pose)
    def lengths(self,p):
        t=pose_matrix(p);pts=self.platform@t[:3,:3].T+t[:3,3];return np.linalg.norm(pts-self.base,axis=1)
    def ik(self,p,seed=None):
        if not np.isfinite(p).all() or p[2]<=.10:raise ValueError('Stewart platform must be above the base.')
        q=self.lengths(p)
        if np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):raise ValueError('Stewart leg stroke exceeded.')
        return q
    def fk(self,q,seed=None):
        seed=self.home_pose if seed is None else pose_values(seed)
        fit=least_squares(lambda p:self.lengths(p)-q,seed,bounds=([-1,-1,.101,-1.3,-1.3,-1.3],[1,1,1.1,1.3,1.3,1.3]),max_nfev=160,gtol=1e-11,xtol=1e-11,ftol=1e-11)
        if np.linalg.norm(fit.fun)>1e-6:raise ValueError('Stewart forward closure did not converge near the seed.')
        return pose_matrix(fit.x)
    def segments_pose(self,t):
        p=self.platform@t[:3,:3].T+t[:3,3]
        return [(self.base[i],p[i]) for i in range(6)]+[(self.base[i],self.base[(i+1)%6]) for i in range(6)]+[(p[i],p[(i+1)%6]) for i in range(6)]
    def segments(self,q):return self.segments_pose(self.fk(q))
    def table(self,q):return [[str(i+1),'P',f'{q[i]:.4f}',f'{self.base[i,0]:.3f}',f'{self.base[i,1]:.3f}',f'{self.platform[i,0]:.3f}, {self.platform[i,1]:.3f}'] for i in range(6)]

def robots():
    pi=np.pi;L=[(-2.6,2.6)]
    return {
      'SCARA':Serial('SCARA',[('R',0,.235,.30,0,1),('R',0,0,.25,0,1),('P',0,0,0,0,-1),('R',0,0,0,0,1)],L*2+[(0,.15),(-pi,pi)],[.3,-.8,.05,0]),
      'Stanford':Serial('Stanford',[('R',0,.30,0,-pi/2,1),('R',0,.12,0,pi/2,1),('P',0,.08,0,0,1),('R',0,0,0,-pi/2,1),('R',0,0,0,pi/2,1),('R',0,.10,0,0,1)],L*2+[(.10,.55)]+L*3,[.3,.5,.3,.2,.4,0]),
      'PUMA':Serial('PUMA',[('R',0,.32,0,pi/2,1),('R',0,0,.32,0,1),('R',0,.08,.04,-pi/2,1),('R',0,.28,0,pi/2,1),('R',0,0,0,-pi/2,1),('R',0,.10,0,0,1)],L*6,[.2,.4,-.6,.2,.5,0]),
      'SCORBOT':Serial('SCORBOT',[('R',0,.30,0,pi/2,1),('R',0,0,.25,0,1),('R',0,0,.22,0,1),('R',0,0,0,pi/2,1),('R',0,.10,0,0,1)],L*5,[.2,.5,-.8,.3,.2]),
      'Delta':Delta(),'Stewart':Stewart()}

def quintic(u):
    u=np.clip(u,0,1);return u**3*(10-15*u+6*u*u)

def plan(robot,start,goals,seconds=4):
    """Joint-space quintic segments; validate sampled parallel closure and actuator rates.
    Stewart uses local FK continuation. No collision planning. Full stop at waypoints.
    """
    if not np.isfinite(seconds) or seconds<=0:raise ValueError('Duration must be positive.')
    result=[];q0=np.array(start);tseed=robot.fk(q0)
    for goal in goals:
        goal=np.array(goal)
        if not np.isfinite(goal).all() or np.any(goal<robot.limits[:,0]) or np.any(goal>robot.limits[:,1]):raise ValueError('Waypoint actuator limit exceeded.')
        delta=np.abs(goal-q0)
        duration=max(seconds,float(np.max(1.875*delta/robot.vmax)),float(np.max(np.sqrt(5.774*delta/robot.amax))))
        poses=[]
        for u in np.linspace(0,1,121):
            q=q0+(goal-q0)*quintic(u)
            t=robot.fk(q,tseed) if isinstance(robot,Stewart) else robot.fk(q)
            tseed=t;poses.append(t)
        result.append((q0.copy(),goal.copy(),duration,poses));q0=goal
    return result

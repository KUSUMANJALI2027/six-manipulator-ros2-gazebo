import numpy as np
from kinematics import *
from gazebo_mirror import beads,world_xml
import xml.etree.ElementTree as ET
rng=np.random.default_rng(30)
count=0
for name,r in robots().items():
    t=r.fk(r.home);q=r.ik(pose_values(t),r.home)
    assert np.linalg.norm(pose_error(r.fk(q),t))<1e-5,(name,'home round trip')
    good=0
    for _ in range(12):
        if isinstance(r,Stewart):
            p=r.home_pose+rng.uniform([-.025,-.025,-.02,-.03,-.03,-.03],[.025,.025,.02,.03,.03,.03]);q=r.ik(p);t=r.fk(q)
            assert np.linalg.norm(pose_error(t,pose_matrix(p)))<1e-5
        else:
            q=np.clip(r.home+rng.uniform(-.07,.07,len(r.home)),r.limits[:,0]+1e-6,r.limits[:,1]-1e-6)
            t=r.fk(q);qi=r.ik(pose_values(t),r.home)
            assert np.linalg.norm(pose_error(r.fk(qi),t))<1e-5,(name,'IK/FK')
        good+=1
    if isinstance(r,Delta):
        e=r.elbows(q);p=r.fk(q)[:3,3]+r.offset
        assert np.max(np.abs(np.linalg.norm(p-e,axis=1)-r.lower))<1e-9
    goals=[r.home.copy(),q,r.home.copy()];segments=plan(r,r.home,goals,1)
    for a,b,d,poses in segments:
        assert np.max(1.875*np.abs(b-a)/d-r.vmax)<1e-10
        assert np.max(5.774*np.abs(b-a)/d**2-r.amax)<1e-10
        assert np.linalg.norm(pose_error(poses[-1],r.fk(b)))<1e-5
        for u in np.linspace(0,1,101):
            x=a+(b-a)*quintic(u);assert np.all(x>=r.limits[:,0]-1e-10) and np.all(x<=r.limits[:,1]+1e-10)
    ET.fromstring(world_xml(beads(r.segments(r.home))))
    try:r.ik(np.array([4,4,4,0,0,0]),r.home)
    except ValueError:pass
    else:raise AssertionError(name+' accepted distant target')
    print(f'PASS {name}: home + {good} pose round trips, path closure, actuator timing and limits, SDF XML, distant-target rejection')
    count+=1
print(f'PASS: {count} educational models. ROS/RViz/Gazebo runtime verification is separate.')

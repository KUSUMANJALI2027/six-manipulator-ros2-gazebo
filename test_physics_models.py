import xml.etree.ElementTree as ET
import numpy as np
from kinematics import robots,Serial,dh
from serial_sdf import world_sdf
rng=np.random.default_rng(44)
for name,r in robots().items():
 if not isinstance(r,Serial):continue
 root=ET.fromstring(world_sdf(r));model=root.find('world').findall('model')[-1]
 assert len(model.findall('joint'))==len(r.home)+1
 assert model.find("joint[@name='mount']/parent").text=='world'
 frames=r.frames(r.home)
 for link in model.findall('link'):
  inertia=link.find('inertial/inertia');vals=[float(inertia.find(k).text) for k in ['ixx','iyy','izz']]
  assert min(vals)>0 and max(vals)<=sum(vals)-max(vals)+1e-9
 for _ in range(80):
  q=rng.uniform(r.limits[:,0],r.limits[:,1]);t=np.eye(4)
  for i,row in enumerate(r.rows):
   joint=model.find(f"joint[@name='joint_{i+1}']")
   axis=np.fromstring(joint.find('axis/xyz').text,sep=' ')
   assert joint.find('axis/xyz').attrib['expressed_in']==('base' if i==0 else f'link_{i}')
   zero_relative=np.linalg.inv(frames[i])@frames[i+1];value=q[i]-r.home[i]
   # SDF child motion around the explicitly parent-relative Z joint axis.
   delta=dh(axis[2]*value,0,0,0) if row[0]=='R' else dh(0,axis[2]*value,0,0)
   t=t@delta@zero_relative
   assert np.max(np.abs(t-r.frames(q)[i+1]))<1e-8,(name,i)
   lo=float(joint.find('axis/limit/lower').text);hi=float(joint.find('axis/limit/upper').text)
   assert lo<=value<=hi
 print(f'PASS {name}: articulated topology, positive inertias, joint-coordinate mapping against DH in 80 random configurations, shifted limits')
print('NOT TESTED: Gazebo parser/plugin loading, controller stability, contact behavior, ROS bridge runtime. First-run verification required.')

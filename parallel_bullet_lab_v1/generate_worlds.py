"""Visual bodies for the custom Bullet system; no standard gz Physics plugin."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from xml.etree.ElementTree import Element,SubElement,ElementTree
from kinematics_reference import Delta,Stewart
ROOT=Path(__file__).resolve().parent

def add(parent,tag,text=None,**attrs):
 e=SubElement(parent,tag,attrs)
 if text is not None:e.text=str(text)
 return e

def main():
 (ROOT/'worlds').mkdir(exist_ok=True);(ROOT/'results').mkdir(exist_ok=True)
 for name,r in [('Delta',Delta()),('Stewart',Stewart())]:
  sdf=Element('sdf',version='1.9');w=add(sdf,'world',name='parallel_bullet')
  phy=add(w,'physics',name='timing',type='ignored');add(phy,'max_step_size','.001');add(phy,'real_time_factor','0')
  add(w,'plugin',filename='gz-sim-scene-broadcaster-system',name='gz::sim::systems::SceneBroadcaster')
  add(w,'plugin',filename='gz-sim-user-commands-system',name='gz::sim::systems::UserCommands')
  scene=add(w,'scene');add(scene,'ambient','.7 .7 .7 1');add(scene,'background','.14 .16 .21 1')
  light=add(w,'light',name='sun',type='directional');add(light,'direction','-.5 -.5 -1');add(light,'diffuse','1 1 1 1')
  m=add(w,'model',name=name);add(m,'static','false');plugin=add(m,'plugin',filename='ParallelBulletSystem',name='parallel_lab::ParallelSystem');add(plugin,'robot',name);add(plugin,'csv_path',ROOT/'results'/f'{name}_gazebo_feedback.csv')
  def body(label,p,radius,length,q=None):
   q=Rotation.identity() if q is None else q;link=add(m,'link',name=label);add(link,'pose',' '.join(map(str,np.r_[p,q.as_euler('xyz')])));v=add(link,'visual',name='body');geo=add(v,'geometry');c=add(geo,'cylinder');add(c,'radius',radius);add(c,'length',length);mat=add(v,'material');color='0.95 0.55 0.10 1' if label=='platform' else '0.12 0.55 0.88 1';add(mat,'ambient',color);add(mat,'diffuse',color)
  def rod(label,a,b,radius):
   d=np.asarray(b)-a;d=d/np.linalg.norm(d);z=np.array([0.,0.,1.]);axis=np.cross(z,d);norm=np.linalg.norm(axis)
   if norm<1e-12:rot=Rotation.identity() if np.dot(z,d)>0 else Rotation.from_rotvec([np.pi,0,0])
   else:rot=Rotation.from_rotvec(axis/norm*np.arccos(np.clip(np.dot(z,d),-1,1)))
   body(label,(a+b)/2,radius,np.linalg.norm(b-a),rot)
  body('fixed_base',[0,0,0],.035,.04)
  if name=='Delta':
   p=r.fk(r.home)[:3,3];e=r.elbows(r.home);body('platform',p,r.rp,.018)
   for i in range(3):
    rod(f'upper_{i}',r.base[i],e[i],.02)
    tangent=np.cross([0,0,1],r.rad[i])*.018
    for side in [-1,1]:rod(f'forearm_{i}_{side+1}',e[i]+side*tangent,p+r.offset[i]+side*tangent,.008)
  else:
   p=r.platform+[0,0,.48];body('platform',[0,0,.48],.20,.025)
   for i in range(6):
    d=p[i]-r.base[i];rod(f'lower_{i}',r.base[i],r.base[i]+.55*d,.014);rod(f'upper_{i}',r.base[i]+.45*d,p[i],.011)
  ElementTree(sdf).write(ROOT/'worlds'/f'{name}.sdf',encoding='utf-8',xml_declaration=True)
 print('Generated Delta and Stewart worlds with absolute output CSV paths.')
if __name__=='__main__':main()

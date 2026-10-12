"""Build articulated serial SDFs from the Six Robot Lab standard DH geometry.
Joint coordinate in Gazebo = DH actuator coordinate - home coordinate.
Masses, inertias and gains are illustrative, not identified hardware parameters.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from kinematics import Serial

def pose(t):return ' '.join(f'{v:.12g}' for v in np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_euler('xyz')])
def model_sdf(r):
    if not isinstance(r,Serial):raise ValueError('Articulated SDF generator supports serial robots only.')
    frames=r.frames(r.home);items=[];names=[]
    items.append('''<link name="base"><inertial><mass>5</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><visual name="base_visual"><pose>0 0 0.025 0 0 0</pose><geometry><box><size>0.18 0.18 0.05</size></box></geometry><material><ambient>0.2 0.25 0.3 1</ambient><diffuse>0.2 0.25 0.3 1</diffuse></material></visual></link><joint name="mount" type="fixed"><parent>world</parent><child>base</child></joint>''')
    for i,(row,frame) in enumerate(zip(r.rows,frames[1:])):
        name=f'link_{i+1}';parent='base' if i==0 else f'link_{i}';joint=f'joint_{i+1}';names.append(joint)
        # Each body's geometry is expressed in its child DH frame.
        end=np.zeros(3);start=(np.linalg.inv(frame)@np.r_[frames[i][:3,3],1])[:3]
        length=float(np.linalg.norm(start));center=start/2;shape=np.eye(4);shape[:3,3]=center
        if length>.015:
            axis=-start/length;z=np.array([0.,0.,1.]);cross=np.cross(z,axis);dot=float(np.dot(z,axis))
            rot=Rotation.from_rotvec(np.array([np.pi,0,0])) if dot<-.999999 else Rotation.from_rotvec(cross/(np.linalg.norm(cross)+1e-20)*np.arccos(np.clip(dot,-1,1)))
            shape[:3,:3]=rot.as_matrix();geometry=f'<cylinder><radius>0.025</radius><length>{length}</length></cylinder>'
            mass=.35;ix=mass*(3*.025**2+length**2)/12;iz=mass*.025**2/2
        else:
            shape=np.eye(4);geometry='<sphere><radius>0.032</radius></sphere>';mass=.12;ix=iz=.4*mass*.032**2
        material='<material><ambient>0.10 0.48 0.85 1</ambient><diffuse>0.10 0.48 0.85 1</diffuse></material>'
        items.append(f'''<link name="{name}"><pose>{pose(frame)}</pose><inertial><pose>{pose(shape)}</pose><mass>{mass}</mass><inertia><ixx>{ix}</ixx><iyy>{ix}</iyy><izz>{iz}</izz></inertia></inertial><visual name="body"><pose>{pose(shape)}</pose><geometry>{geometry}</geometry>{material}</visual><collision name="body_collision"><pose>{pose(shape)}</pose><geometry>{geometry}</geometry></collision><visual name="joint_cap"><geometry><sphere><radius>0.034</radius></sphere></geometry><material><ambient>1 0.55 0.05 1</ambient><diffuse>1 0.55 0.05 1</diffuse></material></visual></link>''')
        kind,theta,d,a,alpha,sign=row;lo,hi=r.limits[i]-r.home[i];typ='revolute' if kind=='R' else 'prismatic'
        items.append(f'''<joint name="{joint}" type="{typ}"><pose relative_to="{parent}">0 0 0 0 0 0</pose><parent>{parent}</parent><child>{name}</child><axis><xyz expressed_in="{parent}">0 0 {sign}</xyz><limit><lower>{lo}</lower><upper>{hi}</upper><effort>100</effort><velocity>{r.vmax[i]}</velocity></limit><dynamics><damping>0.15</damping></dynamics></axis></joint>''')
        items.append(f'''<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController"><joint_name>{joint}</joint_name><topic>/physics_lab/{r.name}/cmd/joint_{i+1}</topic><p_gain>{600 if kind=='P' else 120}</p_gain><i_gain>5</i_gain><d_gain>{30 if kind=='P' else 8}</d_gain><i_max>10</i_max><i_min>-10</i_min><cmd_max>100</cmd_max><cmd_min>-100</cmd_min><initial_position>0</initial_position></plugin>''')
    items.append('<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>/physics_lab/'+r.name+'/feedback</topic><update_rate>30</update_rate>'+''.join(f'<joint_name>{n}</joint_name>' for n in names)+'</plugin>')
    return f'<model name="{r.name}"><self_collide>false</self_collide>'+''.join(items)+'</model>'

def world_sdf(r):
    return '''<?xml version="1.0"?><sdf version="1.9"><world name="serial_physics"><physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><light name="sun" type="directional"><pose>0 0 5 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.5 -0.5 -1</direction></light><scene><ambient>0.65 0.65 0.65 1</ambient><background>0.16 0.18 0.23 1</background></scene><model name="ground"><static>true</static><pose>0 0 -0.06 0 0 0</pose><link name="floor"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry></collision><visual name="floor"><geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry></visual></link></model>'''+model_sdf(r)+'</world></sdf>'

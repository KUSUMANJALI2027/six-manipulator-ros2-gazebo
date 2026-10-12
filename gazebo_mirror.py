"""Gazebo Harmonic geometry mirror using documented set_pose_vector service.
Beads show kinematic link centerlines. No joints, masses, contacts or dynamics.
"""
import subprocess,threading,time
from pathlib import Path
import numpy as np

def beads(segments):return [a+(b-a)*u for a,b in segments for u in np.linspace(0,1,6)]
def world_xml(points):
    models=[]
    for i,p in enumerate(points):
        models.append(f'''<model name="bead_{i}"><static>true</static><pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose><link name="visual"><visual name="ball"><geometry><sphere><radius>0.022</radius></sphere></geometry><material><ambient>0.15 0.55 0.9 1</ambient><diffuse>0.15 0.55 0.9 1</diffuse></material></visual></link></model>''')
    return '''<?xml version="1.0"?><sdf version="1.9"><world name="robot_lab">
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<gravity>0 0 -9.81</gravity><scene><ambient>0.7 0.7 0.7 1</ambient><background>0.15 0.18 0.22 1</background></scene>
<light name="sun" type="directional"><pose>0 0 5 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 -0.5 -1</direction></light>
<model name="ground"><static>true</static><link name="ground"><visual name="ground"><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry><material><ambient>0.5 0.5 0.5 1</ambient></material></visual></link></model>
'''+''.join(models)+'</world></sdf>'

class Mirror:
    def __init__(self,root,segments):
        folder=Path(root)/'results';folder.mkdir(exist_ok=True);p=folder/'gazebo_mirror.sdf';self.points=beads(segments);p.write_text(world_xml(self.points));self.error='';self.done=threading.Event()
        self.log=(folder/'gazebo.log').open('w');self.proc=subprocess.Popen(['gz','sim','-r',str(p)],stdout=self.log,stderr=subprocess.STDOUT)
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def update(self,segments):self.points=beads(segments)
    def run(self):
        while not self.done.wait(.8):
            if self.proc.poll() is not None:self.error='Gazebo exited; see results/gazebo.log';return
            payload=' '.join(f'pose {{ name: "bead_{i}" position {{ x: {float(p[0])} y: {float(p[1])} z: {float(p[2])} }} orientation {{ w: 1 }} }}' for i,p in enumerate(self.points))
            try:
                result=subprocess.run(['gz','service','-s','/world/robot_lab/set_pose_vector','--reqtype','gz.msgs.Pose_V','--reptype','gz.msgs.Boolean','--timeout','500','--req',payload],capture_output=True,text=True,timeout=2)
                self.error='' if result.returncode==0 and 'true' in result.stdout.lower() else 'Gazebo mirror waiting for pose service; see results/gazebo.log'
            except Exception as e:self.error=str(e)
    def stop(self):
        self.done.set()
        if self.proc.poll() is None:self.proc.terminate()
        self.log.close()

#!/usr/bin/env python3
import sys,time,csv,subprocess,os
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,TransformStamped
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,QDoubleSpinBox,QTableWidget,QTableWidgetItem,QCheckBox,QFileDialog,QMessageBox)
from kinematics import robots,Serial,Delta,Stewart,pose_values,pose_matrix,pose_error,plan,quintic
ROOT=Path(__file__).resolve().parent

class Panel(QWidget):
    def __init__(self):
        super().__init__();self.setWindowTitle('Six Robot Lab | Kinematics and trajectories');self.resize(880,840)
        self.node=Node('six_robot_lab');self.pub=self.node.create_publisher(MarkerArray,'/robot_lab/markers',10)
        self.jpub=self.node.create_publisher(JointState,'/robot_lab/joint_states',10);self.tf=TransformBroadcaster(self.node)
        self.models=robots();self.queue=deque();self.motion=None;self.trail=[];self.samples=[];self.waypoints=[];self.children=[];self.records=[];self.gazebo=None
        box=QVBoxLayout(self)
        label=QLabel('SIX ROBOT LAB — educational geometry');label.setStyleSheet('font-size:22px;font-weight:bold;color:#165b94');box.addWidget(label)
        note=QLabel('RViz + optional Gazebo kinematic mirror. No contact/dynamics simulation or collision checking.\nSerial DH: Rz(theta) Tz(d) Tx(a) Rx(alpha). Position: metres; angles: radians in tables.');note.setWordWrap(True);box.addWidget(note)
        self.selector=QComboBox();self.selector.addItems(self.models);box.addWidget(self.selector)
        row=QHBoxLayout();box.addLayout(row)
        for text,fn in [('Home',self.home),('Load demo waypoints',self.demo),('STOP / hold',self.stop),('Open Gazebo mirror',self.open_gazebo)]:
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b)
        self.details=QLabel();self.details.setWordWrap(True);box.addWidget(self.details)
        self.live=QLabel();self.live.setStyleSheet('font-size:16px;background:#e8f6ec;padding:10px');box.addWidget(self.live)
        self.table=QTableWidget();self.table.setMaximumHeight(200);box.addWidget(self.table)
        box.addWidget(QLabel('Joint/actuator values: edit, then Apply joints. Rotation inputs below are degrees.'))
        self.jrow=QHBoxLayout();box.addLayout(self.jrow);self.jinputs=[]
        b=QPushButton('Apply joints smoothly');b.clicked.connect(self.apply_joints);box.addWidget(b)
        row=QHBoxLayout();box.addLayout(row);self.inputs=[]
        for label in ['X','Y','Z','Roll °','Pitch °','Yaw °']:
            col=QVBoxLayout();row.addLayout(col);col.addWidget(QLabel(label));v=QDoubleSpinBox();v.setRange(-180,180) if '°' in label else v.setRange(-2,2);v.setDecimals(4);v.setSingleStep(5 if '°' in label else .01);col.addWidget(v);self.inputs.append(v)
        row=QHBoxLayout();box.addLayout(row)
        for text,fn in [('Use current pose',self.target_current),('Solve IK + move',self.move_target),('Add target waypoint',self.add_target)]:
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b)
        row=QHBoxLayout();box.addLayout(row);self.duration=QDoubleSpinBox();self.duration.setRange(1,60);self.duration.setValue(4);row.addWidget(QLabel('Minimum seconds per segment'));row.addWidget(self.duration)
        for text,fn in [('Run waypoints',self.run_waypoints),('Clear waypoints',self.clear_waypoints),('Import pose CSV',self.import_csv)]:
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b)
        self.wp=QLabel();box.addWidget(self.wp)
        row=QHBoxLayout();box.addLayout(row);self.show_workspace=QCheckBox('Show workspace');self.show_trail=QCheckBox('Show path');self.show_trail.setChecked(True);row.addWidget(self.show_workspace);row.addWidget(self.show_trail)
        for text,fn in [('Sample workspace',self.workspace),('Clear path',self.clear_path),('Export motion CSV',self.export)]:
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b)
        self.status=QLabel('Ready');self.status.setWordWrap(True);box.addWidget(self.status)
        self.selector.currentTextChanged.connect(self.select);self.select('SCARA')
        self.timer=QTimer();self.timer.timeout.connect(self.tick);self.timer.start(40)
    def say(self,s):self.status.setText(s)
    def select(self,name):
        self.stop();self.robot=self.models[name];self.q=self.robot.home.copy();self.t=self.robot.fk(self.q);self.trail=[];self.samples=[];self.records=[];self.waypoints=[];self.wp.setText('Waypoints: 0')
        while self.jrow.count():
            item=self.jrow.takeAt(0)
            if item.widget():item.widget().deleteLater()
        self.jinputs=[]
        for i,(lohi,unit) in enumerate(zip(self.robot.limits,self.robot.units)):
            v=QDoubleSpinBox();lo,hi=lohi;v.setRange(*(np.rad2deg([lo,hi]) if unit=='rad' else [lo,hi]));v.setDecimals(3);v.setPrefix(f'{i+1}: ');v.setSuffix('°' if unit=='rad' else ' m');v.setValue(np.rad2deg(self.q[i]) if unit=='rad' else self.q[i]);self.jrow.addWidget(v);self.jinputs.append(v)
        descriptions={'SCARA':'4 DOF: XYZ + yaw; roll/pitch fixed. Same dimensions as the earlier demo.', 'Stanford':'6 DOF RRP-RRR educational Stanford-type arm.', 'PUMA':'6 DOF RRR-RRR educational PUMA-type arm; dimensions are illustrative.', 'SCORBOT':'5 DOF educational SCORBOT-type arm. Arbitrary 6D target poses may not be feasible.', 'Delta':'3 DOF translation. Parallelogram branches keep platform orientation fixed. Lower assembly branch.', 'Stewart':'6 leg lengths; local numerical FK near the upright assembly. No universal-joint angle limits or leg collision checks.'}
        self.details.setText(descriptions[name]);self.target_current();self.say('Ready. Load demo waypoints → Run waypoints.')
        if self.gazebo:self.gazebo.stop();self.gazebo=None
    def stop(self):
        self.queue.clear();self.motion=None
        if hasattr(self,'status'):self.say('Stopped; holding displayed pose. Stop is immediate, not a physical braking profile.')
    def schedule(self,goals):
        self.stop()
        try:
            segments=plan(self.robot,self.q,goals,self.duration.value());self.queue.extend(segments);self.say(f'{len(segments)} segments validated for joint positions and sampled closure; timing includes configured speed/acceleration limits.')
        except Exception as e:self.say(str(e))
    def home(self):self.schedule([self.robot.home])
    def apply_joints(self):
        q=[np.deg2rad(v.value()) if u=='rad' else v.value() for v,u in zip(self.jinputs,self.robot.units)];self.schedule([q])
    def target_current(self):
        p=pose_values(self.t);p[3:]=np.rad2deg(p[3:])
        for v,a in zip(self.inputs,p):v.setValue(float(a))
    def target(self):
        p=np.array([v.value() for v in self.inputs]);p[3:]=np.deg2rad(p[3:]);return p
    def solve(self,p,seed):
        q=self.robot.ik(p,seed)
        # A local numerical FK must reproduce the requested assembly pose.
        t=self.robot.fk(q) if not isinstance(self.robot,Stewart) else self.robot.fk(q,self.t)
        if np.linalg.norm(pose_error(t,pose_matrix(p)))>1e-4:raise ValueError('Requested pose is not on the current FK assembly branch.')
        return q
    def move_target(self):
        try:self.schedule([self.solve(self.target(),self.q)])
        except Exception as e:self.say(str(e))
    def add_target(self):
        try:
            q=self.solve(self.target(),self.waypoints[-1] if self.waypoints else self.q);self.waypoints.append(q);self.wp.setText(f'Waypoints: {len(self.waypoints)}');self.say('Target added.')
        except Exception as e:self.say(str(e))
    def clear_waypoints(self):self.waypoints=[];self.wp.setText('Waypoints: 0')
    def demo(self):
        self.clear_waypoints();r=self.robot
        if isinstance(r,Stewart):
            poses=[r.home_pose,r.home_pose+np.array([.035,0,.025,.04,0,0]),r.home_pose+np.array([0,.035,0,0,.04,0]),r.home_pose]
            self.waypoints=[r.ik(p) for p in poses]
        else:
            a=r.home.copy();b=a.copy();c=a.copy();b[0]+=.22;c[1]+=.20
            if isinstance(r,Serial) and r.name=='SCARA':b[2]=.09;c[2]=.025
            self.waypoints=[a,b,c,a]
        self.wp.setText(f'Waypoints: {len(self.waypoints)} (demo: home → A → B → home)');self.say('Click Run waypoints. Each waypoint is a full stop.')
    def run_waypoints(self):
        if not self.waypoints:self.say('Load demo waypoints or add targets first.');return
        self.schedule(self.waypoints)
    def import_csv(self):
        filename,_=QFileDialog.getOpenFileName(self,'Pose CSV',str(ROOT/'waypoints'),'CSV (*.csv)')
        if not filename:return
        try:
            found=[];seed=self.q
            with open(filename) as f:
                for row in csv.DictReader(f):
                    p=np.array([float(row[k]) for k in ['x','y','z','roll_deg','pitch_deg','yaw_deg']]);p[3:]=np.deg2rad(p[3:]);seed=self.solve(p,seed);found.append(seed)
            self.waypoints=found;self.wp.setText(f'Waypoints: {len(found)}');self.say('CSV targets solved. Click Run waypoints.')
        except Exception as e:self.say('CSV not loaded: '+str(e))
    def workspace(self):
        if self.motion or self.queue:self.say('Stop motion before sampling workspace.');return
        rng=np.random.default_rng(7);out=[];r=self.robot
        for _ in range(700):
            try:
                if isinstance(r,Stewart):
                    p=np.r_[rng.uniform([-.22,-.22,.30],[.22,.22,.65]),0,0,0];r.ik(p);out.append(p[:3])
                else:
                    q=rng.uniform(r.limits[:,0],r.limits[:,1]);out.append(r.fk(q)[:3,3])
            except ValueError:pass
        self.samples=out;self.show_workspace.setChecked(True);self.say(f'{len(out)} position samples. No collision filtering.'+(' Stewart slice uses zero platform orientation.' if isinstance(r,Stewart) else ' Tool orientation varies for serial robots.'))
    def clear_path(self):self.trail=[]
    def export(self):
        folder=ROOT/'results';folder.mkdir(exist_ok=True);p=folder/f'{self.robot.name}_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        with p.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['elapsed_s']+[f'q{i+1}_{u}' for i,u in enumerate(self.robot.units)]+['x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad']);w.writerows(self.records)
        self.say(f'Saved {len(self.records)} calculated samples: {p}')
    def open_gazebo(self):
        try:
            from gazebo_mirror import Mirror
            if self.gazebo:self.gazebo.stop()
            self.gazebo=Mirror(ROOT,self.segments());self.say('Opening Gazebo kinematic mirror. Allow startup time; it is a geometry display, not dynamics.')
        except Exception as e:self.say('Gazebo could not open: '+str(e))
    def segments(self):return self.robot.segments_pose(self.t) if isinstance(self.robot,Stewart) else self.robot.segments(self.q)
    def marker(self,ident,kind,points,scale,color):
        m=Marker();m.header.frame_id='world';m.header.stamp=self.node.get_clock().now().to_msg();m.ns='robot_lab';m.id=ident;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=float(scale);m.scale.y=float(scale);m.scale.z=float(scale);m.color.r,m.color.g,m.color.b,m.color.a=map(float,color);m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points];return m
    def tick(self):
        if not rclpy.ok():self.close();return
        rclpy.spin_once(self.node,timeout_sec=0)
        try:
            if self.motion is None and self.queue:self.motion=(*self.queue.popleft(),time.monotonic())
            if self.motion:
                a,b,duration,poses,began=self.motion;elapsed=time.monotonic()-began;u=min(1,elapsed/duration);self.q=a+(b-a)*quintic(u)
                self.t=self.robot.fk(self.q,self.t) if isinstance(self.robot,Stewart) else self.robot.fk(self.q)
                self.say(f'Moving {u*100:.0f}% | segment {duration:.1f} s | {len(self.queue)} segments remaining')
                if u>=1:self.motion=None
                if not self.motion and not self.queue:self.say('Waypoints completed. Export motion CSV if needed.')
            p=pose_values(self.t)
            if not self.trail or np.linalg.norm(p[:3]-self.trail[-1])>.001:self.trail.append(p[:3].copy());self.trail=self.trail[-4000:]
            self.records.append([time.monotonic()-START,*self.q,*p]);self.records=self.records[-30000:]
            self.live.setText('Tool: X={:.4f}   Y={:.4f}   Z={:.4f} m\nRoll={:.2f}°   Pitch={:.2f}°   Yaw={:.2f}°'.format(*p[:3],*np.rad2deg(p[3:])))
            headers=['Joint','Type','theta rad','d m','a m','alpha rad'] if isinstance(self.robot,Serial) else (['Branch','Type','angle rad','upper m','lower m','base/platform radius'] if isinstance(self.robot,Delta) else ['Leg','Type','length m','base X','base Y','platform X,Y'])
            rows=self.robot.table(self.q);self.table.setColumnCount(6);self.table.setRowCount(len(rows));self.table.setHorizontalHeaderLabels(headers)
            for i,row in enumerate(rows):
                for j,value in enumerate(row):self.table.setItem(i,j,QTableWidgetItem(value))
            self.table.resizeColumnsToContents()
            seg=self.segments();pts=[p for pair in seg for p in pair];markers=[self.marker(0,Marker.LINE_LIST,pts,.028,(.15,.55,.9,1)),self.marker(1,Marker.SPHERE_LIST,pts,.045,(1,.55,.05,1)),self.marker(2,Marker.POINTS,self.samples if self.show_workspace.isChecked() else [],.008,(.1,.85,.9,.5)),self.marker(3,Marker.LINE_STRIP,self.trail if self.show_trail.isChecked() and len(self.trail)>1 else [],.008,(1,.1,.55,1)),self.marker(4,Marker.SPHERE_LIST,[self.t[:3,3]],.065,(.1,.9,.2,1))]
            for axis,col in enumerate([(1,0,0,1),(0,1,0,1),(0,.4,1,1)]):
                markers.append(self.marker(5+axis,Marker.LINE_LIST,[self.t[:3,3],self.t[:3,3]+.10*self.t[:3,axis]],.008,col))
            for marker in markers:
                if not marker.points:marker.action=Marker.DELETE
            self.pub.publish(MarkerArray(markers=markers));js=JointState();js.header.stamp=self.node.get_clock().now().to_msg();js.name=[f'{self.robot.name}_actuator_{i+1}' for i in range(len(self.q))];js.position=self.q.tolist();self.jpub.publish(js)
            tf=TransformStamped();tf.header.stamp=js.header.stamp;tf.header.frame_id='world';tf.child_frame_id='lab_tool';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.t[:3,3]);qq=Rotation.from_matrix(self.t[:3,:3]).as_quat();tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,qq);self.tf.sendTransform(tf)
            if self.gazebo:
                self.gazebo.update(seg)
                if self.gazebo.error:self.say(self.gazebo.error)
        except Exception as e:self.stop();self.say('Motion stopped: '+str(e))
    def closeEvent(self,event):
        self.timer.stop()
        if self.gazebo:self.gazebo.stop()
        self.node.destroy_node();event.accept()

START=time.monotonic()
if __name__=='__main__':
    rclpy.init();app=QApplication(sys.argv);panel=Panel();panel.show()
    rviz=subprocess.Popen(['rviz2','-d',str(ROOT/'lab.rviz')])
    try:app.exec_()
    finally:
        rviz.terminate()
        if rclpy.ok():rclpy.shutdown()

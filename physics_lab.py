#!/usr/bin/env python3
"""Serial Gazebo physics trial: commands and simulator feedback remain separate."""
import sys,time,subprocess,csv,os,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QComboBox,QPushButton,QLabel,QTableWidget,QTableWidgetItem
from kinematics import robots,Serial,pose_values,plan,quintic
from serial_sdf import world_sdf
ROOT=Path(__file__).resolve().parent

class Lab(QWidget):
    def __init__(self):
        super().__init__();self.setWindowTitle('Gazebo serial physics | command vs feedback');self.resize(820,580)
        self.node=Node('serial_physics_panel');self.children=[];self.logs=[];self.pubs=[];self.sub=None;self.robot=None;self.measured=None;self.motion=[];self.start=None;self.last_feedback=0;self.rows=[];self.first=True;self.sim_time=None;self.last_advance=0
        box=QVBoxLayout(self);box.addWidget(QLabel('ARTICULATED GAZEBO TRIAL — SCARA / Stanford / PUMA / SCORBOT'))
        note=QLabel('Gravity + illustrative rigid bodies + PID joint controllers. Gains need runtime verification.\nFeedback below comes from Gazebo. Delta / Stewart dynamics are not included.');note.setWordWrap(True);box.addWidget(note)
        self.select=QComboBox();self.models={n:r for n,r in robots().items() if isinstance(r,Serial)};self.select.addItems(list(self.models));box.addWidget(self.select)
        row=QHBoxLayout();box.addLayout(row)
        for text,fn in [('Start / restart Gazebo',self.launch),('Run small test',self.demo),('Hold setpoint',self.hold),('Export feedback',self.export)]:
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b)
        self.state=QLabel('Select SCARA, then Start / restart Gazebo.');self.state.setWordWrap(True);box.addWidget(self.state)
        self.table=QTableWidget();box.addWidget(self.table);self.pose=QLabel();self.pose.setWordWrap(True);box.addWidget(self.pose)
        self.timer=QTimer();self.timer.timeout.connect(self.tick);self.timer.start(40)
    def cleanup(self):
        self.motion=[];self.start=None
        for p in self.children:
            if p.poll() is None:p.terminate()
        for p in self.children:
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:p.kill()
        self.children=[]
        for f in self.logs:f.close()
        self.logs=[]
        for p in self.pubs:self.node.destroy_publisher(p)
        self.pubs=[]
        if self.sub:self.node.destroy_subscription(self.sub);self.sub=None
    def launch(self):
        self.cleanup();self.robot=self.models[self.select.currentText()];r=self.robot;self.command=r.home.copy();self.measured=None;self.last_feedback=0;self.rows=[];self.first=True;self.sim_time=None;self.last_advance=0
        out=ROOT/'results';out.mkdir(exist_ok=True);sdf=out/f'{r.name}_physics.sdf';sdf.write_text(world_sdf(r));self.epoch=time.monotonic();self.env=os.environ.copy();self.env["GZ_PARTITION"]="serial_lab_"+uuid.uuid4().hex;self.env["IGN_PARTITION"]=self.env["GZ_PARTITION"]
        args=[]
        for i in range(len(r.home)):
            topic=f'/physics_lab/{r.name}/cmd/joint_{i+1}';self.pubs.append(self.node.create_publisher(Float64,topic,10));args.append(topic+'@std_msgs/msg/Float64]gz.msgs.Double')
        feedback=f'/physics_lab/{r.name}/feedback';args.append(feedback+'@sensor_msgs/msg/JointState[gz.msgs.Model')
        self.sub=self.node.create_subscription(JointState,feedback,self.feedback,10)
        try:
            for label,cmd in [('gazebo',['gz','sim','-r',str(sdf)]),('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',*args])]:
                f=(out/f'{r.name}_{label}.log').open('w');self.logs.append(f);self.children.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,env=self.env))
            self.state.setText('Starting. Wait for live feedback and let the home pose settle before Run small test.')
        except Exception as e:self.cleanup();self.state.setText(str(e))
    def feedback(self,msg):
        if self.robot is None:return
        values=dict(zip(msg.name,msg.position));keys=[f'joint_{i+1}' for i in range(len(self.robot.home))]
        # Some bridges preserve scoped joint names.
        values={k.split('::')[-1]:v for k,v in values.items()}
        if not all(k in values for k in keys):return
        measured=np.array([values[k] for k in keys])+self.robot.home
        if not np.isfinite(measured).all():self.state.setText('Invalid feedback; stop Gazebo and send its log.');return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.sim_time is not None and stamp<self.sim_time:
            self.hold();self.state.setText('Simulation clock reset. Restart this panel before another test.');return
        if self.sim_time is None or stamp>self.sim_time:self.last_advance=time.monotonic()
        self.sim_time=stamp
        self.measured=measured;self.last_feedback=time.monotonic()
        p=pose_values(self.robot.fk(measured));self.rows.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,*self.command,*measured,*p]);self.rows=self.rows[-30000:]
        if self.first:self.first=False;self.state.setText('Gazebo feedback received. Wait until joint errors settle, then Run small test.')
    def demo(self):
        if self.measured is None or time.monotonic()-self.last_feedback>1:self.state.setText('Wait for fresh Gazebo feedback first.');return
        try:
            a=self.command.copy();b=a.copy();b[0]+=.15;c=a.copy();c[1]+=.12
            self.motion=plan(self.robot,a,[b,c,a],4);self.start=None;self.state.setText('Running small joint test; compare command and measured values below.')
        except Exception as e:self.state.setText(str(e))
    def hold(self):self.motion=[];self.start=None;self.state.setText('Holding last commanded setpoint. Controller remains active.')
    def tick(self):
        if not rclpy.ok():self.close();return
        rclpy.spin_once(self.node,timeout_sec=0)
        if self.robot is None or not self.pubs:return
        if any(p.poll() is not None for p in self.children):self.hold();self.state.setText('Gazebo or bridge exited. Send results/'+self.robot.name+'_gazebo.log and _bridge.log.');return
        fresh=self.measured is not None and time.monotonic()-self.last_feedback<1 and time.monotonic()-self.last_advance<2
        if self.motion and not fresh:self.hold();self.state.setText('Feedback missing/stale: trajectory cancelled; last setpoint held.')
        if self.motion:
            if self.start is None:self.start=self.sim_time
            a,b,d,poses=self.motion[0];u=min(1,(self.sim_time-self.start)/d);self.command=a+(b-a)*quintic(u)
            if u>=1:self.motion.pop(0);self.start=None
            if not self.motion:self.state.setText('Command sequence finished. Check measured errors after settling; then Export feedback.')
        for pub,value in zip(self.pubs,self.command-self.robot.home):pub.publish(Float64(data=float(value)))
        if not fresh:
            if time.monotonic()-self.epoch>15:self.state.setText('No fresh feedback. Check Gazebo is playing; send results/'+self.robot.name+'_bridge.log.')
            return
        self.table.setRowCount(len(self.command));self.table.setColumnCount(4);self.table.setHorizontalHeaderLabels(['Joint (unit)','Command','Gazebo measured','Error'])
        for i,(cmd,actual,unit) in enumerate(zip(self.command,self.measured,self.robot.units)):
            for j,value in enumerate([f'{i+1} ({unit})',f'{cmd:.5f}',f'{actual:.5f}',f'{actual-cmd:+.5f}']):self.table.setItem(i,j,QTableWidgetItem(value))
        self.table.resizeColumnsToContents();p=pose_values(self.robot.fk(self.measured));self.pose.setText('Tool position reconstructed from measured Gazebo joint states:\nX={:.4f} m   Y={:.4f} m   Z={:.4f} m'.format(*p[:3]))
    def export(self):
        if self.robot is None or not self.rows:self.state.setText('No feedback samples yet.');return
        p=ROOT/'results'/f'{self.robot.name}_physics_feedback_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        with p.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['simulation_time_s']+[f'cmd_{i+1}_{u}' for i,u in enumerate(self.robot.units)]+[f'measured_{i+1}_{u}' for i,u in enumerate(self.robot.units)]+['tool_x','tool_y','tool_z','roll_rad','pitch_rad','yaw_rad']);w.writerows(self.rows)
        self.state.setText('Saved '+str(p))
    def closeEvent(self,event):self.timer.stop();self.cleanup();self.node.destroy_node();event.accept()

if __name__=='__main__':
    rclpy.init();app=QApplication(sys.argv);window=Lab();window.show()
    try:app.exec_()
    finally:
        if rclpy.ok():rclpy.shutdown()

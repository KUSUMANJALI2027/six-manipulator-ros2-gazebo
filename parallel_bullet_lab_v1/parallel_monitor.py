"""Publish actual custom-Bullet feedback from the plugin CSV into ROS 2."""
import csv,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def main():
 robot=sys.argv[1];path=Path(sys.argv[2]);rclpy.init();node=Node('parallel_bullet_feedback')
 joints=node.create_publisher(JointState,f'/parallel_lab/{robot}/measured_joints',10)
 poses=node.create_publisher(PoseStamped,f'/parallel_lab/{robot}/tool_pose',10)
 cmdpub=node.create_publisher(JointState,f'/parallel_lab/{robot}/command_joints',10)
 f=None;fields=None;last_print=0
 try:
  while rclpy.ok():
   if f is None:
    if not path.exists():time.sleep(.1);continue
    f=path.open();fields=f.readline().strip().split(',')
    if not fields or fields[0]!='simulation_time_s':f.close();f=None;time.sleep(.1);continue
   start=f.tell();line=f.readline()
   if not line or not line.endswith('\n'):f.seek(start);time.sleep(.02);continue
   values=line.strip().split(',')
   if len(values)!=len(fields):continue
   row=dict(zip(fields,map(float,values)));t=row['simulation_time_s'];n=3 if robot=='Delta' else 6;unit='rad' if robot=='Delta' else 'm'
   measured=JointState();measured.header.stamp.sec=int(t);measured.header.stamp.nanosec=int((t-int(t))*1e9);measured.name=[f'{robot}_actuator_{i+1}' for i in range(n)];measured.position=[row[f'measured_{i+1}_{unit}'] for i in range(n)];joints.publish(measured)
   command=JointState();command.header=measured.header;command.name=measured.name;command.position=[row[f'cmd_{i+1}_{unit}'] for i in range(n)];cmdpub.publish(command)
   p=PoseStamped();p.header=measured.header;p.header.frame_id='world';p.pose.position.x=row['tool_x'];p.pose.position.y=row['tool_y'];p.pose.position.z=row['tool_z'];p.pose.orientation.x=row['qx'];p.pose.orientation.y=row['qy'];p.pose.orientation.z=row['qz'];p.pose.orientation.w=row['qw'];poses.publish(p)
   now=time.monotonic()
   if now-last_print>.5:
    error=max(abs(a-b) for a,b in zip(command.position,measured.position));print(f'{robot} t={t:.2f}s  max joint error={error:.6g} {unit}  anchor gap={row["max_anchor_gap_m"]*1000:.4f} mm  tool=({row["tool_x"]:.4f}, {row["tool_y"]:.4f}, {row["tool_z"]:.4f})',flush=True);last_print=now
   rclpy.spin_once(node,timeout_sec=0)
 except KeyboardInterrupt:pass
 finally:
  if f:f.close()
  node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

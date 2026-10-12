# Six-Manipulator Robotics Laboratory — ROS 2 and Gazebo

**Author:** Vemula Kusumanjali · B.Tech ECE, CUTM Vizianagaram

SCARA, Stanford-type, PUMA-type, SCORBOT-type, Delta and Stewart educational models: forward/inverse kinematics, workspace sampling, quintic waypoint trajectories, RViz visualization and recorded Gazebo/Bullet physics trials.

## Download the report and results
- [Project report (PDF)](reports/Project_Report.pdf)
- [Editable Overleaf source](documentation/overleaf/)
- [Recorded simulator feedback](evidence/)
- [Workspace plots, trajectories and metrics](reports/)
- [Parallel physics source and build instructions](parallel_bullet_lab_v1/)

## Quick start
On Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:
```bash
git clone https://github.com/KUSUMANJALI2027/six-manipulator-ros2-gazebo.git
cd six-manipulator-ros2-gazebo
bash setup.sh
bash start.sh
```
Read [QUICK_START.md](QUICK_START.md) and [PHYSICS_README.md](PHYSICS_README.md). Build the parallel add-on using its README before using the Delta/Stewart physics viewers. The main GUI and geometry mirror are distinct from measured physics feedback.

## Scope
Simulation-only educational geometries; no physical hardware validation. Recorded serial trials excite J1/J2; parallel trials exercise the prescribed small motions. These results do not establish arbitrary payload, collision or industrial performance. The proposed UR5e adaptive-workspace research is future work.

## Reproduce analysis

Use Ubuntu system Python with numpy, scipy and matplotlib. If needed: `sudo apt install python3-matplotlib`.

```
python3 validate_release.py
python3 analyze_feedback.py
python3 export_six_robot_results.py
```

For new feedback recordings, put one CSV per robot in a separate directory, then run `python3 analyze_feedback.py DIRECTORY --output OUTPUT_DIRECTORY`.

## Verified scope and limitations

Delta and Stewart have mathematical FK/IK, sampled workspace, trajectory generation, RViz display and optional Gazebo geometry mirror. Their custom native-Bullet Gazebo System plugin has now compiled and passed the prescribed small-motion trials on the user WSL machine. The original feedback and screenshots are included under evidence/parallel and evidence/screenshots. Four serial robots have recorded basic physics trials; other configurations, payloads, all-joint excitation and collision behavior are not established by those trials. Educational dimensions are not vendor specifications.

The updated physics_lab.py uses simulator timestamps for trajectory progress and a private Gazebo transport partition. Its Python syntax and mathematical components were checked here; its new process-launch behavior still requires the user's ROS/Gazebo environment. Original evidence was recorded with the earlier runtime.

## Parallel physics demonstration

The already-built add-on is at `~/robotics_project/parallel_bullet_lab_v1`. Run `bash run_gazebo.sh Delta` or `bash run_gazebo.sh Stewart` from that directory. Both headless trials are already verified; no repeat is needed to recreate the included report. This release includes the same add-on source in `parallel_bullet_lab_v1/` for a fresh installation.

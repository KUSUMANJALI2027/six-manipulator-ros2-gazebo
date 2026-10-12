# Six Robot Lab

A runnable educational kinematics project for ROS 2 Jazzy on Ubuntu 24.04 / WSL.
One window selects SCARA, Stanford-type, PUMA-type, SCORBOT-type, Delta or Stewart.
This package is a new standalone lab; it does not modify the previous SCARA files.

## Fast start

Place `robotics_suite.zip` in `C:\robotics`. In Ubuntu:

```bash
python3 -m zipfile -e /mnt/c/robotics/robotics_suite.zip ~/robotics_project
cd ~/robotics_project/robotics_suite
bash setup.sh
bash run.sh
```

Stop earlier robot/IK/trajectory launchers first to reduce RAM usage. Keep only this lab open.
After first setup, only `cd ~/robotics_project/robotics_suite && bash run.sh` is needed.

1. In the control window select SCARA.
2. Click **Load demo waypoints**, then **Run waypoints**.
3. Click **Sample workspace** after movement finishes.
4. Repeat for Stanford, PUMA, SCORBOT, Delta and Stewart.
5. **Export motion CSV** saves calculated actuator and tool data under `results/`.
6. **Open Gazebo mirror** optionally opens the currently selected geometry in Gazebo.

RViz: blue segments are links, orange spheres are connection points, green is tool center,
red/green/blue short lines are tool X/Y/Z axes, pink is the traveled tool path, cyan is workspace sampling.

## Included features

- Explicit, editable standard-DH serial models (including joint types, offsets and limits).
- Serial FK by homogeneous transforms; constrained numerical full-pose IK.
- Delta: 3 actuated upper arms, equal-length paired lower links, fixed-orientation platform;
  analytical sphere-intersection FK and analytical per-branch IK, lower assembly mode.
- Stewart: six leg lengths from platform pose; local numerical FK near the previous upright pose.
- Live actuator values, live serial DH / parallel geometry tables and calculated tool pose.
- Numerical IK residual check, limits and unreachable-target/solver-failure messages.
- Joint-space quintic interpolation through a sequence of waypoints, stopping at each waypoint.
- Per-actuator speed and acceleration limits determine minimum segment duration.
- Parallel loop closure checked at 121 samples per planned segment; runtime FK continuation.
- Joint-input mode, editable XYZ/RPY target mode, CSV waypoints, default demos for all six robots.
- Position workspace sampling and tool path tracing.
- ROS MarkerArray, JointState and world → lab_tool TF output.
- Optional Gazebo kinematic mirror using Gazebo Harmonic's set_pose_vector service.
- Automated numerical checks and six example waypoint CSVs / six SDF geometry snapshots.

## What is NOT complete

This is **not a completed dynamics / collision planning project**. Gazebo renders externally computed
poses as bead centerlines; its SDF files contain independent static visual models, not an articulated
physical mechanism. It does not simulate masses, torques, contacts, grasping or actuator feedback.
The Gazebo mirror refreshes more slowly than RViz. Actual articulated Gazebo physics models,
controller tuning, collision-free planning, and measured simulation tracking errors remain work.
A running Gazebo window by itself does not establish those features.

All serial dimensions are illustrative educational geometry, not manufacturer-accurate hardware.
Stanford is RRP-RRR, PUMA is RRR-RRR with spherical wrist, SCORBOT is a 5-axis articulated model.
Use the instructor's DH tables/dimensions if supplied. `model_parameters.json` and `kinematics.py`
state the exact geometry and conventions used. SCARA uses the prior 0.30/0.25 m arms and
`z = 0.235 - q3`. SCORBOT cannot satisfy every 6D pose; Delta controls only XYZ.

Numerical IK failure means **no solution was found**, not mathematical proof that none exists.
Stewart forward kinematics is assembly-dependent; use small motions around the supplied home.
Spherical/universal-joint angle bounds, singularity avoidance and leg/link collision checks are absent.
The Delta parallelograms are idealized; passive bearing motion is not dynamically modeled.
Workspace points are estimates: no volume/integration guarantee or collision filtering.
Stewart workspace is a fixed-orientation slice, while serial samples allow orientation to vary.
Joint-space interpolation generally creates a curved tool path; it is not straight-line Cartesian planning.
The STOP button freezes the displayed pose immediately and does not represent physical braking.

## Waypoint CSV format

```csv
x,y,z,roll_deg,pitch_deg,yaw_deg
0.30,0.20,0.15,0,0,0
```

Use the example CSV for the selected robot. Targets are relative to `world`, using metres and degrees.
Import solves all poses before accepting the list. Run validates sampled closure before movement.
Current seed and assembly constraints may reject an otherwise geometrically possible target.

## Files

- `app.py`: ROS + Qt control window and RViz marker publication.
- `kinematics.py`: six models, FK/IK, geometry, quintic trajectory planning.
- `gazebo_mirror.py`: optional kinematic visualization in Gazebo.
- `lab.rviz`: topic preconfigured to `/robot_lab/markers` (avoids the earlier empty-topic error).
- `setup.sh`, `run.sh`: dependency install and launch.
- `test_models.py`: headless numerical verification.
- `model_parameters.json`, `dh_tables/`: exact learning-model parameters.
- `waypoints/`: known-feasible example tool poses.
- `worlds/`: six SDF geometry snapshots; not dynamics models.
- `VALIDATION.txt`: checks run while preparing this package.

## Troubleshooting

- No RViz points: check Fixed Frame=`world` and MarkerArray Topic=`/robot_lab/markers`.
- No GUI after WSL restart: use the working WSLg session; run from Ubuntu, not Windows Python.
- Import error: rerun `bash setup.sh`; `run.sh` deliberately uses `/usr/bin/python3`.
- Gazebo exits or stays blank: inspect `results/gazebo.log`; close other Gazebo instances.
- Gazebo stops refreshing: check whether simulation is paused and whether the pose service is ready.
- ROS/RViz/Gazebo runtime was unavailable in the build environment. Numerical checks passed;
  integrated GUI and Gazebo behavior still require a first run on your machine.

## References

Gazebo Harmonic UserCommands, including `set_pose_vector`, request `gz.msgs.Pose_V`, response `gz.msgs.Boolean`:
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1UserCommands.html

Rotation calculations and bounded least-squares use SciPy. DH convention and all geometry are explicitly
specified in this package; no claim of manufacturer-model fidelity is made.

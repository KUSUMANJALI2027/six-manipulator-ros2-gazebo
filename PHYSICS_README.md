# Serial Gazebo physics add-on — first-run trial

This extends the existing Six Robot Lab with articulated SCARA, Stanford-type, PUMA-type
and SCORBOT-type models. Extract its files directly into `~/robotics_project/robotics_suite`.
The original `app.py`, kinematics, datasets and RViz demo are unchanged.

## Launch

Close earlier Gazebo instances and the previous Six Robot Lab control window.

```bash
cd ~/robotics_project/robotics_suite
bash run_physics.sh
```

1. Leave SCARA selected; click **Start / restart Gazebo**.
2. Wait for **Gazebo feedback received** and observe the measured-value table.
3. Allow approximately 5 seconds for the home setpoint to settle.
4. Click **Run small test**. The first two joints move through small offsets and return.
5. Let errors settle, then **Export feedback**.
6. Change the selector and click Start / restart Gazebo to test another serial robot.

If feedback does not arrive, inspect/send `results/SCARA_gazebo.log` and
`results/SCARA_bridge.log`. A visible Gazebo model is not proof of working control;
the table must receive simulator joint states and respond to command changes.
The window deliberately labels command and measured simulation values separately.

## What changed

- Dynamic links with positive cylinder/sphere inertias and illustrative masses.
- Base fixed to world, revolute/prismatic joints and joint position/velocity/effort limits.
- Gravity enabled, body collision geometry and a floor. Self-collision disabled.
- Gazebo Harmonic JointPositionController PID plugins, using force commands (no pose teleport).
- ROS Float64 → Gazebo Double command bridges; Gazebo Model → ROS JointState feedback bridge.
- JointStatePublisher, command-versus-feedback table and timestamped CSV recording.
- Graceful hold of the last command when simulator feedback becomes stale during a trajectory.
- Controller stays active after Hold setpoint; this is not a hardware emergency stop.

Gazebo joint coordinates are relative to the model's initial home assembly:
`gazebo_joint = model_joint - home_joint`. The explicit parent-frame joint axes include
the negative-Z SCARA prismatic sign. Display and export restore the original coordinates.
The feedback-derived tool pose is calculated with FK, not an independent world-pose sensor.
Commands use wall-clock timing, while feedback records Gazebo simulation timestamps.
If simulation runs slowly or pauses, timing/tracking can differ from commanded timing.

## Validation and limitations

Tested locally: SDF XML structure, articulated topology, positive inertias, position bounds,
and joint-coordinate geometry against DH FK for 80 random configurations per serial model.

**Not yet runtime-tested:** Gazebo parser/plugin compatibility, force-controller stability,
ROS bridge delivery, tracking errors and ground contacts. These require the user's installed
ROS Jazzy / Gazebo Harmonic environment. Gains are initial trial values and may need tuning.
Do not describe the generated files as a validated dynamic model or claim measured accuracy
until actual simulator feedback has been examined. Exported CSVs provide that evidence.

Dimensions, masses and gains remain educational assumptions. Robot-body shapes are simple
cylinders/spheres and may not match real hardware envelopes. Prismatic bodies use fixed-size
visuals rather than a detailed telescoping assembly. No obstacle-avoiding path planner,
self-collision validation, inverse dynamics or realistic motor model is included.

**Delta and Stewart are still kinematic-only in the existing lab.** Their closed-loop dynamic
mechanisms are not implemented by this serial add-on. The full six-robot project remains
unfinished until those requirements and final validation/documentation are resolved.

## Sources

Gazebo Harmonic JointPositionController:
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html

Gazebo Harmonic JointStatePublisher:
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html

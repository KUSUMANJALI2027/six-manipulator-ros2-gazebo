# Parallel Bullet Lab — first Gazebo integration trial

This standalone add-on implements **constrained rigid-body dynamics** for the educational Delta and Stewart geometries. It does not change `robotics_suite` or the four working serial-robot trials.

## What changed

The built-in Gazebo engine adapters rejected the tested closed-loop mechanism. This add-on therefore owns a native Bullet 3.24 dynamics world inside a custom Gazebo System plugin. The standard Gazebo Physics system is intentionally absent from these worlds. The plugin integrates mass, inertia, gravity, actuator forces/torques and constraints, then sends the resulting body transforms to Gazebo's visual entities. Commanded joint positions are never imposed as body poses during stepping.

This is a **custom Bullet physics system integrated with Gazebo**, not an extension of the stock gz-physics adapter. It is also not the earlier FK-only Gazebo mirror. The same C++ dynamics core can run headless for numerical checking.

## Mechanisms and assumptions

- Delta: three driven upper-arm hinges; six passive lower rods attached through point constraints in paired parallelograms; a rigid moving platform. Hinge stops match the reference actuator range. Upper arms: 0.20 kg each; lower rods: 0.06 kg each; platform: 0.30 kg.
- Stewart: six spherical-prismatic-spherical telescoping legs, matching the reference anchor locations and length ranges. Each half-leg is 0.15 kg; platform is 1 kg. This is an SPS educational realization; universal-joint bearing-angle limits are not included.
- Inertias are calculated from cylinders. Gravity is 9.81 m/s². Contacts and self-collisions are disabled. Fixed attachments carry ideal world reactions. No payload robustness or hardware fidelity is claimed.
- Delta control: PID torque with P=50, I=15, D=2.5 and +/-15 Nm saturation.
- Stewart control: PID axial force with P=2500, I=600, D=60 and +/-100 N saturation.
- Integral state is bounded; gains are educational. Tests cover small prescribed motions around home, not arbitrary poses.
- 1 ms internal steps; 120 Bullet solver iterations. Large simulator timestep or reset causes the plugin to stop with a clear diagnostic.

## First run on the user's machine

Save the ZIP to `C:\robotics`, extract into `~/robotics_project`, enter its `parallel_bullet_lab_v1` directory, and run:

```
bash build_and_test.sh
```

The script uses the already installed compiler, CMake, Bullet and ROS Gazebo vendor libraries. It installs nothing, modifies no old robot files, and uses an isolated Gazebo transport partition. It builds the core and Gazebo plugin, runs both core trials, then runs two headless Gazebo trials. The GUI is not required at this stage. A result ZIP is created and copied to `C:\robotics` on success or failure. Upload that ZIP to review actual integration evidence. A failed build must not be reported as a physics pass.

The core trials cover 51 simulated seconds for Delta and 87 for Stewart. Gazebo trials run slightly longer to retain a final hold. Every actuator moves individually: +0.04 rad for each Delta input, +0.004 m for each Stewart leg, returning to home. This is small-motion coverage only.

## View after the trial passes

```
bash run_gazebo.sh Delta
# or
bash run_gazebo.sh Stewart
```

The viewer uses real-time pacing; automatic motion starts after a five-second hold. Terminal feedback shows command error, anchor gap and actual platform position. Close the Gazebo window to stop. Restart instead of using simulator reset.

ROS 2 topics published by the CSV feedback reader:

- `/parallel_lab/Delta/measured_joints` (or Stewart): `sensor_msgs/JointState`.
- `/parallel_lab/Delta/command_joints`: `sensor_msgs/JointState`.
- `/parallel_lab/Delta/tool_pose`: `geometry_msgs/PoseStamped`.

The timestamps are simulator time. The platform pose is read from the simulated rigid body, not reconstructed from FK. Gazebo writes 50 Hz CSV feedback; the optional ROS reader republishes those samples. This is not hard-real-time control.

## Verification and limits of evidence

`verify_feedback.py` compares the independently simulated platform pose with the reference FK calculated from measured actuator coordinates. It also checks finite data, increasing timestamps, every actuator's command variation, final tracking error, recorded constraint gaps and model limits.

The investigation gates are stated in source: <1 mm sampled anchor/slider gap, <1 mm sampled FK position disagreement, <0.01 rad sampled orientation disagreement; final five-second tracking <0.01 rad (Delta) or <1 mm (Stewart). These are developer-selected screening gates, not instructor-approved or industrial accuracy specifications. CSV sampling does not prove the absence of between-sample peaks. The core's console reports its per-step maximum gap as an additional check.

`reference_core_results/` contains actual native Bullet 3.24 results generated in the assistant's Linux build environment. It includes a two-second actuator-disabled gravity control experiment. Those results **do not establish that the Gazebo plugin compiled or ran on the user's WSL**. Gazebo integration is the purpose of the new trial.

## Sources

- Gazebo 8 System plugin interfaces and build pattern: https://gazebosim.org/api/sim/8/createsystemplugins.html
- Gazebo 8 EntityComponentManager API: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1EntityComponentManager.html
- Bullet source / rigid-body constraints: https://github.com/bulletphysics/bullet3
- Exact project geometry: `kinematics_reference.py`, copied from the uploaded current project.

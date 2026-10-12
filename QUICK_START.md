# Six Robot Lab — consolidated release

This release combines the existing six-robot kinematics project, four serial physics models, original simulator recordings, reproducible analysis, and project report. It is installed in a separate directory and does not overwrite the existing project.

## Start

Run `bash start.sh` from this folder. Choose 1 for the six-robot kinematics interface or 2 for the serial Gazebo physics interface. The existing ROS Jazzy / Gazebo installation is used. Close previous instances of these labs before starting another.

The main GUI: select robot > Load demo waypoints > Run waypoints > Sample workspace. The optional Open Gazebo mirror button displays calculated geometry; it does not provide parallel-robot physics.

Choose 4 to open the report. If WSL cannot open the PDF, open the separately supplied PDF directly in Windows.

## Evidence already included

- Four ORIGINAL recorded Gazebo feedback CSVs under evidence/.
- Four command/feedback plots and per-joint metrics under reports/.
- Six calculated trajectory CSVs and workspace images under reports/.
- Numerical validation results and source scripts.
- Editable report DOCX, PDF, and explanation notes.

Do not repeat the four earlier basic trials merely to recreate these plots. The optional all-joint coverage test is provided for broader validation; it has not been run on the user's simulator in the supplied evidence.

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

#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
printf '\nSix Robot Lab\n1. Six-robot kinematics (RViz)\n2. Four-robot Gazebo physics\n3. Optional all-joint coverage trial\n4. Open project report\n5. Numerical verification (no ROS required)\n6. Delta custom physics viewer\n7. Stewart custom physics viewer\n'
read -r -p 'Choose [1]: ' choice
case "${choice:-1}" in
  1) exec bash run.sh ;;
  2) exec bash run_physics.sh ;;
  3) source /opt/ros/jazzy/setup.bash; exec /usr/bin/python3 physics_all_joints.py ;;
  4) if command -v wslview >/dev/null; then exec wslview reports/Project_Report.pdf; else exec xdg-open reports/Project_Report.pdf; fi ;;
  5) exec /usr/bin/python3 validate_release.py ;;
  6|7) parallel_dir="$HOME/robotics_project/parallel_bullet_lab_v1"
       if [ ! -f "$parallel_dir/build/libParallelBulletSystem.so" ]; then parallel_dir="$PWD/parallel_bullet_lab_v1"; fi
       if [ ! -f "$parallel_dir/build/libParallelBulletSystem.so" ]; then printf 'Build the included parallel_bullet_lab_v1 first with bash build_and_test.sh.\n'; exit 1; fi
       if [ "$choice" = 6 ]; then robot=Delta; else robot=Stewart; fi
       exec bash "$parallel_dir/run_gazebo.sh" "$robot" ;;
  *) printf 'Choose a number from 1 to 7.\n'; exit 1 ;;
esac

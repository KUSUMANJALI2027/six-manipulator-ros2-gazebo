#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mkdir -p results
exec > >(tee results/build_and_test.log) 2>&1
finish() {
  code=$?
  /usr/bin/python3 collect_results.py "$code" || true
}
trap finish EXIT
/usr/bin/python3 - <<'PYCODE'
from pathlib import Path
import datetime
p=Path('results');old=[f for f in p.iterdir() if f.is_file() and f.suffix in ['.csv','.json','.log'] and f.name!='build_and_test.log']
if old:
 d=p/('history_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));d.mkdir()
 for f in old:f.rename(d/f.name)
PYCODE
source /opt/ros/jazzy/setup.bash
# ROS vendor prefixes contain the actual gz headers and CMake configurations.
for vendor in /opt/ros/jazzy/opt/*; do
  if [ -d "$vendor" ]; then export CMAKE_PREFIX_PATH="$vendor:${CMAKE_PREFIX_PATH:-}"; fi
done
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="parallel_lab_$(date +%s)_$$"
export IGN_PARTITION="$GZ_PARTITION"
printf 'Building isolated custom Bullet plugin; existing robotics_suite is unchanged.\n'
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
/usr/bin/python3 generate_worlds.py
printf '\nTesting native Bullet core for both parallel robots...\n'
./build/parallel_core Delta results/Delta_core_feedback.csv
./build/parallel_core Stewart results/Stewart_core_feedback.csv
/usr/bin/python3 verify_feedback.py results
cp results/verification.json results/core_verification.json
printf '\nTesting Gazebo integration (headless; no GUI required)...\n'
# max runtime is a guard, not a stability acceptance criterion.
timeout 240 gz sim -s -r --iterations 60000 -v 3 worlds/Delta.sdf > results/Delta_gazebo.log 2>&1
timeout 240 gz sim -s -r --iterations 96000 -v 3 worlds/Stewart.sdf > results/Stewart_gazebo.log 2>&1
/usr/bin/python3 verify_feedback.py results --suffix _gazebo_feedback.csv
cp results/verification.json results/gazebo_verification.json
printf '\nBOTH PARALLEL TRIALS PASSED THE DECLARED INVESTIGATION GATES.\nThese are educational models, with robot contacts disabled.\nView: bash run_gazebo.sh Delta   or   bash run_gazebo.sh Stewart\n'

#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y python3-numpy python3-scipy python3-pyqt5 ros-jazzy-rviz2 ros-jazzy-tf2-ros ros-jazzy-visualization-msgs ros-jazzy-sensor-msgs ros-jazzy-python-qt-binding ros-jazzy-ros-gz
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 test_models.py
printf '\nSetup complete. Run: bash run.sh\n'

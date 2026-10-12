#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
robot="${1:-Delta}"
case "$robot" in Delta|Stewart) ;; *) echo 'Choose Delta or Stewart'; exit 2;; esac
source /opt/ros/jazzy/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="parallel_view_$(date +%s)_$$"
export IGN_PARTITION="$GZ_PARTITION"
/usr/bin/python3 - "$robot" <<'PY'
import sys
from pathlib import Path
import xml.etree.ElementTree as E
r=sys.argv[1];root=Path.cwd();t=E.parse(root/'worlds'/f'{r}.sdf')
t.find('.//real_time_factor').text='1'
p=root/'results'/f'{r}_live_feedback.csv'
if p.exists():p.rename(p.with_name(p.stem+'_previous.csv'))
t.find('.//csv_path').text=str(p)
t.write(root/'worlds'/f'{r}_view.sdf')
PY
/usr/bin/python3 parallel_monitor.py "$robot" "$PWD/results/${robot}_live_feedback.csv" &
monitor_pid=$!
trap 'kill "$monitor_pid" 2>/dev/null || true' EXIT
gz sim -r "worlds/${robot}_view.sdf"

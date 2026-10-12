"""Reproduce numerical checks and record the actual environment and outcomes."""
import datetime, json, platform, subprocess, sys
from pathlib import Path
root=Path(__file__).resolve().parent
out=root/'reports';out.mkdir(exist_ok=True)
records=[]
for name in ['test_models.py','test_physics_models.py']:
    r=subprocess.run([sys.executable,str(root/name)],cwd=root,capture_output=True,text=True,timeout=240)
    records.append({'test':name,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
    print(r.stdout,end='')
report={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'python':sys.version,'platform':platform.platform(),'scope':'Numerical and XML checks only; no ROS or Gazebo runtime executed by this script.','checks':records}
(out/'numerical_validation.json').write_text(json.dumps(report,indent=2))
sys.exit(any(r['exit_code'] for r in records))

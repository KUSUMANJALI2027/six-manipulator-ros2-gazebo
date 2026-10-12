import datetime,json,sys,zipfile,shutil
from pathlib import Path
root=Path(__file__).resolve().parent;out=root/'results';out.mkdir(exist_ok=True)
(out/'run_status.json').write_text(json.dumps({'exit_code':int(sys.argv[1]),'utc':datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2))
target=out/f'parallel_trial_results_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.zip'
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
 for p in out.iterdir():
  if p.suffix in ['.csv','.json','.log']:z.write(p,p.name)
for p in [Path('/mnt/c/robotics')]:
 if p.is_dir():shutil.copy2(target,p/target.name);print('Windows result ZIP: C:\\robotics\\'+target.name)
print('Result ZIP:',target)

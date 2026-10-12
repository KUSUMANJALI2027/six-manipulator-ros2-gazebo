#!/usr/bin/env python3
"""Read-only build inventory for a Gazebo parallel-constraint extension.
Does not launch simulations, install packages or modify existing project source.
"""
import datetime,json,os,shutil,subprocess,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def command(args):
    try:
        p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15)
        return {'command':args,'exit_code':p.returncode,'output':p.stdout[-60000:]}
    except (OSError,subprocess.TimeoutExpired) as e:return {'command':args,'error':str(e)}

def main():
    report={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'python':sys.version,
        'ros_distro':os.environ.get('ROS_DISTRO','not sourced'),'commands':[], 'paths':{}}
    for args in [['gz','sim','--versions'],['cmake','--version'],['g++','--version'],['pkg-config','--list-all']]:
        result=command(args)
        if args[0]=='pkg-config' and 'output' in result:
            result['output']='\n'.join(x for x in result['output'].splitlines() if any(s in x.lower() for s in ['gz-','dart','bullet','sdformat','ignition','eigen']))
        report['commands'].append(result)
    report['commands'].append(command(['dpkg-query','-W','-f=${Package}\t${Version}\n','ros-jazzy-gz-*','libdart*','libbullet*','libgz-*','libsdformat*']))
    prefixes=[Path('/opt/ros/jazzy/opt'),Path('/usr/include'),Path('/usr/lib/x86_64-linux-gnu')]
    patterns=['gz_*_vendor/include/*','gz_physics_vendor/lib/gz-physics-*/engine-plugins/*','dart','bullet','cmake/dart*','cmake/bullet*']
    for parent in prefixes:
        for pat in patterns:
            found=sorted(str(p) for p in parent.glob(pat))
            if found:report['paths'][str(parent/pat)]=found
    # Include only specifically named project/engine diagnostics, never arbitrary home files.
    originals=Path.home()/'robotics_project'/'robotics_suite'
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out=ROOT/'results';out.mkdir(exist_ok=True)
    target=out/f'parallel_build_info_{stamp}.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('build_inventory.json',json.dumps(report,indent=2))
        for name in ['kinematics.py','serial_sdf.py','physics_lab.py']:
            p=originals/name
            if p.is_file():z.write(p,'current_project/'+name)
        windows=Path('/mnt/c/robotics')
        for pattern in ['bullet_loop_report.txt','parallel_physics_check_*.txt']:
            for p in sorted(windows.glob(pattern))[-5:]:
                if p.stat().st_size<2_000_000:z.write(p,'previous_diagnostics/'+p.name)
    print('Saved:',target)
    windows=Path('/mnt/c/robotics')
    if windows.is_dir():
        destination=windows/target.name;shutil.copy2(target,destination);print('Windows copy: C:\\robotics\\'+target.name)
    print('Upload this ZIP. No simulations were run and no existing project source was changed.')
if __name__=='__main__':main()

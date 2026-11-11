"""Exercise the real C++ resume loader in file-only mode, without devices."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture',type=Path,required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        partial = root/'data.csv.partial.test'
        output = root/'output.csv'
        path = Path(str(partial)+'.path.json')
        path.write_text(json.dumps({'schema_version':1,'joint_unit':'deg','pose_count':6,
            'center_joints_deg':[0]*7,
            'points':[{'index':i+1,'joints_deg':[0]*7} for i in range(6)],
            'workspace_constraints':{'minimum_body_z_m':.3,'body_spheres':[
                {'joint_index':6,'center_m':[0,0,.1],'radius_m':.1,'name':'tip'}]}}))
        header = ('# generated_by=force_sensor_calibration_capture\n'
                  '# automatic=true; robot_motion=MoveJ\n'
                  '# sensor_to_tool_rpy_deg=0,0,30\n# probe_to_sensor_rpy_deg=0,0,0\n'
                  'r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,fy,fz,tx,ty,tz\n')
        def samples(count):
            return header+''.join(f'# pose={i+1};\n1,0,0,0,1,0,0,0,1,0,0,-5,0,0,0\n' for i in range(count))
        command = [str(args.capture.resolve()),'--check-resume','--resume',str(partial),
                   '--poses','6','--output',str(output),'--sensor-to-tool-rpy-deg','0,0,30']
        def check(content,expected,diagnostic):
            partial.write_text(content)
            original = partial.read_bytes(),path.read_bytes()
            result = subprocess.run(command,capture_output=True,text=True,timeout=5)
            assert result.returncode==expected,(result.stdout,result.stderr)
            assert diagnostic in result.stdout+result.stderr,(result.stdout,result.stderr)
            assert (partial.read_bytes(),path.read_bytes())==original
            assert sorted(p.name for p in root.iterdir())==sorted([partial.name,path.name])
        check(samples(2),0,'preserved 2 poses; next pose 3/6')
        check(header,5,'zero accepted samples')
        check(samples(6),5,'already contains all planned samples')
        check(samples(2).replace('# automatic=true; robot_motion=MoveJ\n',''),5,'missing automatic capture metadata')
        check(samples(2)[:-1],5,'incomplete final line')
    print('Resume validation: populated/empty/complete/malformed checkpoints passed; no device access.')


if __name__ == '__main__':
    main()

"""Synthetic outlier recovery plus offline production fitting/deployment checks."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import fit_force_robust as robust


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fit',type=Path,required=True);args=parser.parse_args()
    rng=np.random.default_rng(23);n=160
    rotations=np.array([np.linalg.qr(m)[0] for m in rng.normal(size=(n,3,3))])
    rotations[:,:,0]*=np.linalg.det(rotations)[:,None]
    gravity=np.array([.1,-.2,-3.]);bias=np.array([-.7,.3,.2]);com=np.array([.01,-.02,.06]);tbias=np.array([.03,-.04,.02])
    g=np.einsum('nji,j->ni',rotations,gravity)
    wrench=np.column_stack((g+bias,np.cross(com,g)+tbias))
    wrench+=rng.normal(size=(n,6))*np.array([.005]*3+[.0005]*3)
    bad=rng.choice(n,40,replace=False)
    wrench[bad,:3]+=rng.normal(0,10,(40,3));wrench[bad,3:]+=rng.normal(0,.4,(40,3))
    model,mask,_,_,_=robust.robust_fit(rotations,wrench,trials=1000)
    assert mask.sum()==120 and not mask[bad].any()
    assert np.max(np.abs(model[0][:3]-gravity))<.005
    assert np.max(np.abs(model[0][3:]-bias))<.005
    assert np.max(np.abs(model[1][:3]-com))<.001
    again=robust.robust_fit(rotations,wrench,trials=1000)
    assert np.array_equal(mask,again[1])
    try:robust.robust_fit(np.tile(np.eye(3),(10,1,1)),wrench[:10],trials=10)
    except ValueError:pass
    else:raise AssertionError('Unexcited poses must fail')
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);source=root/'samples.csv.partial.synthetic';calibration=root/'calibration.json';out=root/'results'
        geometry={'probe_model':'Lprobe-IFS.STL','sensor_to_tool':{'rotation_row_major':[1,0,0,0,1,0,0,0,1],
            'translation_sensor_to_tool_m':[0,0,-.0125]},'probe_tcp_sensor_m':[0,0,.188]}
        calibration.write_text(json.dumps(geometry));old=calibration.read_bytes()
        with source.open('w') as f:
            f.write('# generated_by=force_sensor_calibration_capture\n# sensor_to_tool_rpy_deg=0,0,0\n# probe_to_sensor_rpy_deg=0,0,0\n')
            f.write('r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,fy,fz,tx,ty,tz\n')
            for i,(r,w) in enumerate(zip(rotations,wrench)):
                f.write(f'# pose={i+1};\n'+','.join(format(v,'.17g') for v in np.r_[r.ravel(),w])+'\n')
        original=source.read_bytes()
        result=subprocess.run([sys.executable,str(Path(robust.__file__)), '--input',str(source),
            '--calibration',str(calibration),'--output-dir',str(out),'--trials','1000','--fit-binary',str(args.fit.resolve()),'--deploy'],
            capture_output=True,text=True,timeout=30)
        assert result.returncode==0,result.stdout+result.stderr
        report=json.loads((out/'report.json').read_text());deployed=json.loads(calibration.read_text())
        assert report['inlier_count']==120 and report['outlier_count']==40 and report['deployed']
        assert deployed['quality']['residuals_verified']
        assert source.read_bytes()==original and (out/'previous_calibration.json').read_bytes()==old
        assert not report['capture_marked_complete'] and b'# capture_complete=true' not in (out/'inliers.csv').read_bytes()
        assert deployed['sensor_to_tool']['translation_sensor_to_tool_m']==[0,0,-.0125]
        current=calibration.read_bytes()
        failed_dir=root/'failed_fit'
        failed=subprocess.run([sys.executable,str(Path(robust.__file__)), '--input',str(source),
            '--calibration',str(calibration),'--output-dir',str(failed_dir),'--trials','1000',
            '--fit-binary','/bin/false','--deploy'],capture_output=True,text=True,timeout=30)
        assert failed.returncode!=0 and calibration.read_bytes()==current
        assert not json.loads((failed_dir/'report.json').read_text())['deployed']
    print('Robust calibration recovered known parameters, rejected injected outliers, preserved source, and passed production fit.')


if __name__=='__main__':main()

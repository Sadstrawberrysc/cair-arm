"""Run production calibration startup validation without hardware or Redis."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser=argparse.ArgumentParser()
    for name in ('controller','calibration','probe'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();profile=json.loads(args.calibration.read_text())
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp);calibration=root/'calibration.json'
        def check(data,expected,message):
            calibration.write_text(json.dumps(data))
            result=subprocess.run([str(args.controller.resolve()),'--check-force-calibration',
                '--calibration',str(calibration),'--probe-model',str(args.probe.resolve()),
                '--robot-ip','127.0.0.1','--robot-port','1','--sensor-device','/nonexistent/kwr57'],
                capture_output=True,text=True,timeout=5)
            assert result.returncode==expected,result.stdout+result.stderr
            assert message in result.stdout+result.stderr,result.stdout+result.stderr
            if expected==0:assert hashlib.sha256(calibration.read_bytes()).hexdigest() in result.stdout
        check(profile,0,'no device connections or motion commands')
        bad=copy.deepcopy(profile);bad['sensor_id']='OTHER';check(bad,3,'sensor_id does not match')
        bad=copy.deepcopy(profile);bad['quality']['residuals_verified']=False;check(bad,3,'verified residuals')
        bad=copy.deepcopy(profile);bad['tool_chain_verified']=False;check(bad,3,'verified residuals')
        bad=copy.deepcopy(profile);bad['probe_model_sha256']='0'*64;check(bad,3,'does not match selected STL')
        bad=copy.deepcopy(profile);bad['quality']['force_max_n']=1.;check(bad,3,'residual limits are not satisfied')
        assert sorted(p.name for p in root.iterdir())==['calibration.json']
    print('Production force calibration: valid profile accepted, invalid identities/quality/geometry rejected, no device I/O.')


if __name__=='__main__':main()

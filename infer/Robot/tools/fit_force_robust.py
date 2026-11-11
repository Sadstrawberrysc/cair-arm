#!/usr/bin/env python3
"""Offline RANSAC rejection followed by the production static-gravity fit."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from calibrate_force_static import BUILD, ROBOT, rpy_deg, vector


def skew(v):
    z=np.zeros(len(v))
    return np.array([[z,-v[:,2],v[:,1]],[v[:,2],z,-v[:,0]],[-v[:,1],v[:,0],z]]).transpose(2,0,1)


def fit_model(rotations,wrenches,ids):
    r=rotations[ids];y=wrenches[ids];n=len(ids)
    a=np.concatenate((r.transpose(0,2,1),np.tile(np.eye(3),(n,1,1))),axis=2).reshape(-1,6)
    if n<6 or np.linalg.cond(a)>20: raise ValueError('Insufficient force orientation excitation')
    force=np.linalg.lstsq(a,y[:,:3].ravel(),rcond=None)[0]
    magnitude=np.linalg.norm(force[:3])
    if not .1<=magnitude<=50: raise ValueError('Unusable identified gravity')
    g=np.einsum('nji,j->ni',r,force[:3])
    b=np.concatenate((-skew(g),np.tile(np.eye(3),(n,1,1))),axis=2).reshape(-1,6)
    normalized=np.concatenate((-skew(g/magnitude),np.tile(np.eye(3),(n,1,1))),axis=2).reshape(-1,6)
    if np.linalg.cond(normalized)>20: raise ValueError('Insufficient torque orientation excitation')
    torque=np.linalg.lstsq(b,y[:,3:].ravel(),rcond=None)[0]
    if np.linalg.norm(torque[:3])>.5: raise ValueError('Unusable center of mass')
    return force,torque


def residuals(rotations,wrenches,model):
    force,torque=model
    g=np.einsum('nji,j->ni',rotations,force[:3])
    predicted_force=g+force[3:]
    predicted_torque=np.cross(torque[:3],g)+torque[3:]
    return np.linalg.norm(predicted_force-wrenches[:,:3],axis=1),np.linalg.norm(predicted_torque-wrenches[:,3:],axis=1)


def robust_fit(rotations,wrenches,trials=4000,seed=20261109,force_limit=.6,torque_limit=.1):
    n=len(rotations)
    if n<6: raise ValueError('At least six samples required')
    rng=np.random.default_rng(seed);best=None;best_score=(-1,-float('inf'))
    for _ in range(trials):
        try: model=fit_model(rotations,wrenches,rng.choice(n,6,replace=False))
        except (ValueError,np.linalg.LinAlgError): continue
        ef,et=residuals(rotations,wrenches,model)
        keep=(ef<=force_limit)&(et<=torque_limit)
        # Maximum joint force/torque consensus; truncated residual breaks ties.
        loss=np.minimum((ef/force_limit)**2+(et/torque_limit)**2,2).sum()
        score=(int(keep.sum()),-float(loss))
        if score>best_score: best_score=score;best=keep
    if best is None or best.sum()<6: raise ValueError('No sufficiently excited consensus found')
    # Refit and remove whole poses, never individual axes. Monotonic rejection
    # terminates and guarantees the final production least-squares gate agrees.
    history=[]
    for _ in range(n+1):
        model=fit_model(rotations,wrenches,np.flatnonzero(best))
        ef,et=residuals(rotations,wrenches,model)
        revised=best&(ef<=force_limit)&(et<=torque_limit)
        history.append({'retained':int(best.sum()),'removed_after_refit':int((best&~revised).sum())})
        if np.array_equal(best,revised): return model,best,ef,et,history
        if revised.sum()<6: raise ValueError('Too few inliers after refinement')
        best=revised
    raise ValueError('Robust fit did not converge')


def read_capture(path):
    raw=path.read_bytes()
    if not raw.endswith(b'\n'): raise ValueError('CSV has an incomplete trailing line')
    rows=[];poses=[];metadata=[];pose=None
    for line in raw.decode().splitlines():
        if line.startswith('# pose='): pose=int(line.split(';')[0].split('=')[1])
        elif line.startswith('#') and not line.startswith(('# resumed_from=','# precheck_')):
            metadata.append(line)
        if not line or line.startswith(('#','r00,')): continue
        values=[float(v) for v in line.split(',')]
        if len(values)!=15 or not np.isfinite(values).all(): raise ValueError('Invalid numeric capture row')
        rows.append(values);poses.append(pose if pose is not None else len(rows))
    a=np.asarray(rows)
    if len(a)<6: raise ValueError('Not enough samples')
    r=a[:,:9].reshape(-1,3,3)
    if np.max(np.abs(r.transpose(0,2,1)@r-np.eye(3)))>1e-5 or np.max(np.abs(np.linalg.det(r)-1))>1e-5:
        raise ValueError('Capture contains invalid rotations')
    return raw,a,poses,metadata


def statistics(values):
    return {'rms':float(np.sqrt(np.mean(values**2))),'max':float(values.max()),
            'median':float(np.median(values)),'p95':float(np.quantile(values,.95))}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,default=BUILD/'rm75_force_calibration.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--trials',type=int,default=4000)
    parser.add_argument('--seed',type=int,default=20261109)
    parser.add_argument('--sensor-id',default='KWR57')
    parser.add_argument('--fit-binary',type=Path,default=BUILD/'force_sensor_calibrate')
    parser.add_argument('--deploy',action='store_true',help='质量门限通过后原子替换部署JSON，保留备份')
    args=parser.parse_args()
    if args.trials<1: parser.error('--trials must be positive')
    raw,a,poses,metadata=read_capture(args.input)
    old=args.calibration.read_bytes();geometry=json.loads(old)
    r=a[:,:9].reshape(-1,3,3);w=a[:,9:]
    model,mask,ef,et,history=robust_fit(r,w,args.trials,args.seed)
    all_model=fit_model(r,w,np.arange(len(a)));before_f,before_t=residuals(r,w,all_model)
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    clean=out/'inliers.csv';candidate=out/'force_calibration.json'
    with clean.open('w') as f:
        for line in metadata: f.write(line+'\n')
        f.write(f'# robust_source={args.input.resolve()}\n# robust_method=RANSAC_joint_wrench_then_QR\n')
        f.write('r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,fy,fz,tx,ty,tz\n')
        for index in np.flatnonzero(mask):
            f.write(f'# pose={poses[index]}; source_row={index+1}\n')
            f.write(','.join(format(v,'.17g') for v in a[index])+'\n')
    report={'source':str(args.input.resolve()),'source_sha256':hashlib.sha256(raw).hexdigest(),
        'capture_marked_complete':b'# capture_complete=true' in raw,'input_count':len(a),
        'inlier_count':int(mask.sum()),'outlier_count':int((~mask).sum()),'retained_fraction':float(mask.mean()),
        'method':'deterministic 6-pose RANSAC, joint force/torque consensus, monotonic whole-pose residual rejection, production QR refit',
        'seed':args.seed,'trials':args.trials,'force_limit_n':.6,'torque_limit_nm':.1,
        'before_all_samples_force_n':statistics(before_f),'before_all_samples_torque_nm':statistics(before_t),
        'after_inliers_force_n':statistics(ef[mask]),'after_inliers_torque_nm':statistics(et[mask]),
        'after_all_samples_force_n':statistics(ef),'after_all_samples_torque_nm':statistics(et),
        'removed_source_poses':[poses[i] for i in np.flatnonzero(~mask)],'iterations':history,
        'validation_note':'Residuals on selected inliers are training residuals, not independent validation; outliers are statistical rejections, not confirmed contact events.',
        'deployed':False}
    with (out/'residuals.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['source_row','source_pose','retained','force_residual_n','torque_residual_nm','reason'])
        for i in range(len(a)):
            reason=[]
            if ef[i]>.6:reason.append('force_residual')
            if et[i]>.1:reason.append('torque_residual')
            if not mask[i] and not reason:reason.append('outside_initial_consensus')
            writer.writerow([i+1,poses[i],bool(mask[i]),ef[i],et[i],';'.join(reason)])
    sr=rpy_deg(geometry['sensor_to_tool']['rotation_row_major'])
    pr=rpy_deg(geometry.get('probe_to_sensor',{}).get('rotation_row_major',[1,0,0,0,1,0,0,0,1]))
    cmd=[str(args.fit_binary.resolve()),'--input',str(clean),'--output',str(candidate),
         '--allow-incomplete','--sensor-id',args.sensor_id,'--probe-model',geometry['probe_model'],
         '--probe-file',str(ROBOT/'model'/geometry['probe_model']),
         '--sensor-to-tool-rpy-deg',sr,'--probe-to-sensor-rpy-deg',pr,
         '--sensor-to-tool-translation-m',vector(geometry['sensor_to_tool']['translation_sensor_to_tool_m']),
         '--probe-tcp-sensor-m',vector(geometry['probe_tcp_sensor_m'])]
    if geometry.get('tool_chain_verified'):cmd.append('--tool-chain-verified')
    result=subprocess.run(cmd,capture_output=True,text=True)
    (out/'production_fit.log').write_text(result.stdout+result.stderr)
    report['production_fit_exit_code']=result.returncode
    if args.input.read_bytes()!=raw: raise RuntimeError('Source changed while fitting; nothing deployed')
    if result.returncode==0 and args.deploy:
        fitted=json.loads(candidate.read_text())
        if not fitted['quality']['residuals_verified']: raise RuntimeError('Candidate quality is unverified')
        if args.calibration.read_bytes()!=old: raise RuntimeError('Deployment JSON changed while fitting')
        backup=out/'previous_calibration.json';backup.write_bytes(old)
        with tempfile.NamedTemporaryFile(dir=args.calibration.parent,prefix='.force-calibration-',delete=False) as f:
            temp=Path(f.name);f.write(candidate.read_bytes());f.flush();os.fsync(f.fileno())
        os.chmod(temp,args.calibration.stat().st_mode & 0o777)
        try:os.replace(temp,args.calibration)
        finally:
            if temp.exists():temp.unlink()
        report['deployed']=True;report['backup']=str(backup)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('input_count','inlier_count','outlier_count','retained_fraction',
        'after_inliers_force_n','after_inliers_torque_nm','production_fit_exit_code','deployed')},indent=2))
    print(f'Report: {out / "report.json"}')
    if result.returncode:raise SystemExit(result.returncode)


if __name__=='__main__':main()

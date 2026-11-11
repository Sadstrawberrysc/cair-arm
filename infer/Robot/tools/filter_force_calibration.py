#!/usr/bin/env python3
"""Filter remaining calibration targets offline; preserve the accepted CSV prefix."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from calibrate_force_static import ROBOT, BUILD, checkpoint_sample_count
from force_calibration_geometry import body_spheres


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='新任务CSV输出；不能覆盖已有文件')
    parser.add_argument('--start-joints-deg',help='离线假定起点的七轴角度；不会移动机械臂')
    args=parser.parse_args()
    source=args.resume.resolve(); output=args.output.resolve()
    target=Path(str(output)+'.partial.filtered')
    snapshot=Path(str(target)+'.path.json'); report=Path(str(output)+'.self_collision_report.json')
    if output.exists() or target.exists() or snapshot.exists() or report.exists():
        parser.error('输出已存在，请选择新的 --output，原任务不会被覆盖')
    original=source.read_bytes(); original_path=Path(str(source)+'.path.json').read_bytes()
    plan=json.loads(original_path); count=checkpoint_sample_count(source)
    if not 0<count<len(plan['points']): parser.error('需要含已采样本和剩余目标的断点')
    config={'urdf':str(ROBOT.parent/'Collision/rm_75.urdf'),
            'mesh_directory':str(ROBOT.parent/'Collision/mesh'),
            'exclusions_file':str(ROBOT.parent/'Collision/config/monitor.example.json'),
            'clearance_m':.005}
    if args.start_joints_deg:
        values=[float(v) for v in args.start_joints_deg.split(',')]
        if len(values)!=7: parser.error('起点需要七个关节角')
        config['start_joints_deg']=values
    # Use the replacement mesh, keeping the original URDF mounting transform.
    bodies=[b for b in body_spheres(ROBOT) if b['joint_index']==6]
    bodies += [b for b in plan['workspace_constraints']['body_spheres'] if b['name']=='calibrated_probe_tcp']
    plan['workspace_constraints']['body_spheres']=bodies
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='force-filter-',dir=output.parent) as temp:
        temp=Path(temp); input_path=temp/'input.json';cfg=temp/'config.json'
        filtered=temp/'filtered.json';rep=temp/'report.json'
        input_path.write_text(json.dumps(plan));cfg.write_text(json.dumps(config))
        result=subprocess.run([str(BUILD/'force_calibration_filter'),str(input_path),str(count),str(cfg),str(filtered),str(rep)],
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        if result.returncode:
            if rep.exists():
                with report.open('x') as f: f.write(rep.read_text())
            raise RuntimeError(result.stderr.strip() or 'No remaining motion passed the filter')
        final=json.loads(filtered.read_text());audit=json.loads(rep.read_text())
        final['self_collision_filter_report']=str(report)
        final['filtered_from']=str(source)
        final['source_csv_sha256']=hashlib.sha256(original).hexdigest()
        final['source_path_sha256']=hashlib.sha256(original_path).hexdigest()
        if source.read_bytes()!=original or Path(str(source)+'.path.json').read_bytes()!=original_path:
            raise RuntimeError('源断点在检查期间发生变化；请先停止采集再重新筛选')
        with snapshot.open('x') as f: f.write(json.dumps(final,indent=2)+'\n')
        with report.open('x') as f: f.write(json.dumps(audit,indent=2)+'\n')
        with target.open('xb') as f: f.write(original)
    print(f'已保留 {count} 个已采样本；剩余保留 {audit["retained_remaining"]} 点，剔除 {audit["removed_count"]} 点。')
    print(f'新总数：{final["pose_count"]}；新断点：{target}\n报告：{report}')
    print('只完成离线筛选，没有连接或控制机械臂。')


if __name__=='__main__':
    main()

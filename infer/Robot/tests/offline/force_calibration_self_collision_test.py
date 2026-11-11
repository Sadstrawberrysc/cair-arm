"""File-only synthetic self-collision/filter tests. No robot/network/device I/O."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    p=argparse.ArgumentParser();p.add_argument('--filter',type=Path,required=True)
    p.add_argument('--capture',type=Path,required=True);a=p.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp);urdf=root/'model.urdf';policy=root/'policy.json';config=root/'config.json'
        # Camera at radius 1: q7=0 intersects base; +/-180 are individually
        # clear, but their joint-linear connecting segment crosses the base.
        xml='<robot name="self_filter_fixture"><link name="base_link"><collision><origin xyz="1 0 0"/><geometry><sphere radius="0.1"/></geometry></collision></link>'
        previous='base_link'
        for i in range(1,8):
            name='camera_link' if i==7 else f'link{i}'
            origin='1 0 0' if i==7 else f'0 0 {-i*2}'
            xml+=f'<link name="{name}"><collision><origin xyz="{origin}"/><geometry><sphere radius="0.1"/></geometry></collision></link>'
            xml+=f'<joint name="joint{i}" type="revolute"><parent link="{previous}"/><child link="{name}"/><axis xyz="0 0 1"/><limit lower="-6.28" upper="6.28" velocity="1"/></joint>'
            previous=name
        urdf.write_text(xml+'</robot>');policy.write_text('{"self_collision_exclusions":[]}')
        cfg={'urdf':str(urdf),'mesh_directory':str(root),'exclusions_file':str(policy),'clearance_m':.005}
        config.write_text(json.dumps(cfg))
        points=[{'index':i+1,'joints_deg':[0]*6+[angle]} for i,angle in enumerate([180]*6+[0,-180,170])]
        plan={'schema_version':1,'joint_unit':'deg','pose_count':9,'points':points,
              'center_joints_deg':[0]*6+[180], 'mode':'workspace_coverage',
              'workspace_constraints':{'minimum_body_z_m':.3,'body_spheres':[
                  {'joint_index':6,'name':'camera','center_m':[0,0,.1],'radius_m':.1}]}}
        source=root/'input.json';source.write_text(json.dumps(plan))
        filtered=root/'filtered.json';report=root/'report.json'
        result=subprocess.run([str(a.filter.resolve()),str(source),'6',str(config),str(filtered),str(report)],capture_output=True,text=True,timeout=15)
        assert result.returncode==0,result.stderr
        final=json.loads(filtered.read_text());r=json.loads(report.read_text())
        assert final['points'][:6]==points[:6]
        assert [v['original_index'] for v in final['points'][6:]]==[9]
        assert [v['original_index'] for v in r['removed']]==[7,8]
        assert r['removed'][0]['reason']['status']=='collision',r
        assert r['removed'][1]['reason']['status']!='valid',r
        assert final['pose_count']==7 and not final['environment_collision_checked']
        partial=root/'data.csv.partial.test'
        Path(str(partial)+'.path.json').write_text(filtered.read_text())
        partial.write_text('# generated_by=force_sensor_calibration_capture\n# automatic=true; robot_motion=MoveJ\n'
            '# sensor_to_tool_rpy_deg=0,0,30\n# probe_to_sensor_rpy_deg=0,0,0\n'
            'r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,fy,fz,tx,ty,tz\n'+
            ''.join(f'# pose={i+1};\n1,0,0,0,1,0,0,0,1,0,0,-5,0,0,0\n' for i in range(6)))
        cmd=[str(a.capture.resolve()),'--check-resume','--resume',str(partial),'--poses','7',
             '--output',str(root/'output.csv'),'--sensor-to-tool-rpy-deg','0,0,30']
        checked=subprocess.run(cmd,capture_output=True,text=True,timeout=10)
        assert checked.returncode==0,checked.stderr
        urdf.write_text(urdf.read_text()+'\n')
        rejected=subprocess.run(cmd,capture_output=True,text=True,timeout=10)
        assert rejected.returncode==5 and 'geometry/policy changed' in rejected.stderr,rejected.stderr
        assert not (root/'output.csv').exists()
        policy.write_text(json.dumps({'self_collision_exclusions':[{'first_link':'camera_link',
            'second_link':'base_link','reason':'forbidden test exclusion'}]}))
        rejected=subprocess.run(cmd,capture_output=True,text=True,timeout=10)
        assert rejected.returncode==5 and 'cannot be excluded' in rejected.stderr,rejected.stderr
    print('Self collision: endpoint and interior rejection, prefix preservation, guard resume and model identity passed.')


if __name__=='__main__':main()

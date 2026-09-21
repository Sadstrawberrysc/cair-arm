#!/usr/bin/env python3
"""Hardware-free Redis/Robot integration check. Requires built main_rm75, numpy, redis-py.
Starts only its own loopback Redis and explicit --simulate Robot; never uses port 7777.
"""
import argparse,csv,json,socket,subprocess,tempfile,time
from pathlib import Path
import numpy as np
import redis
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--no-force",action="store_true")
parser.add_argument("--continuous",action="store_true")
parser.add_argument("--candidate-trial",action="store_true")
parser.add_argument("--unlimited-excursion",action="store_true")
args=parser.parse_args()
ROOT=Path(__file__).resolve().parents[3]
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
folder=Path(tempfile.mkdtemp(prefix='wrist_loopback_'))
redis_log=(folder/'redis.log').open('w');robot_log=(folder/'robot.log').open('w')
server=subprocess.Popen(['redis-server','--bind','127.0.0.1','--port',str(port),'--save','','--appendonly','no','--dir',str(folder)],stdout=redis_log,stderr=subprocess.STDOUT)
robot=None
try:
 r=redis.Redis(host='127.0.0.1',port=port,socket_timeout=.2)
 for _ in range(100):
  try:
   if r.ping():break
  except redis.RedisError:time.sleep(.02)
 sub=r.pubsub(ignore_subscribe_messages=True);sub.subscribe('robot:wrist:state:v1')
 command=[str(ROOT/'infer/Robot/build/main_rm75'),'--simulate','--redis-enabled','--redis-port',str(port),'--duration-sec','0' if args.continuous else '15','--publish-every','1','--wrist-follow',str(ROOT/'infer/Camera_wrist/gemini305_to_rm75_armtip.json'),'--calibration',str(ROOT/'infer/Robot/build/rm75_force_calibration.json'),'--probe-model',str(ROOT/'infer/Robot/model/Lprobe-IFS.STL'),'--runtime-log',str(folder/'runtime.csv')]
 if args.no_force:command.append("--wrist-no-force")
 if args.candidate_trial:command.append("--wrist-candidate-trial")
 if args.unlimited_excursion:command.append("--wrist-unlimited-excursion")
 robot=subprocess.Popen(command,stdout=robot_log,stderr=subprocess.STDOUT)
 state=None; deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  msg=sub.get_message(timeout=.1)
  if msg and msg['type']=='message':state=json.loads(msg['data']);break
  if robot.poll() is not None:raise RuntimeError((folder/'robot.log').read_text())
 assert state,'no seed-free Robot state'
 assert not r.exists('robot:wrist:seed:v1')
 cal=json.loads((ROOT/'infer/Camera_wrist/gemini305_to_rm75_armtip.json').read_text())
 force=json.loads((ROOT/'infer/Robot/build/rm75_force_calibration.json').read_text())
 arm_camera=np.array(cal['T_armtip_camera']);base_camera=np.array(state['T_base_camera']);base_arm=base_camera@np.linalg.inv(arm_camera)
 mount=np.array(force['sensor_to_tool']['rotation_row_major']).reshape(3,3)
 tool_rotation=base_arm[:3,:3]@mount
 tcp=base_arm[:3,3]+base_arm[:3,:3]@mount@np.array(force['probe_tcp_sensor_m'])
 normal=-tool_rotation[:,2];skin=tcp+np.array([.008 if args.unlimited_excursion else .001,0,0])-.05*normal
 seq=0;events=[];capture=0;history=[state];capture_state=state
 def envelope(target):
  global seq
  seq+=1
  return dict(version=1,runtime_session_id=state['runtime_session_id'],producer_id='loopback',target_id=target,sequence=seq,timestamp_monotonic_ns=time.monotonic_ns(),boot_id=state['boot_id'],calibration_sha256=state['calibration_sha256'],frame='gemini305_color_optical',length_unit='m')
 def obs(target,old=False):
  global capture,capture_state
  packet=envelope(target)
  if not old:
   eligible=[s for s in history if s['timestamp_monotonic_ns']<=time.monotonic_ns()-40_000_000]
   capture_state=eligible[-1] if eligible else history[0]
   capture=capture_state['timestamp_monotonic_ns']
  bc=np.array(capture_state['T_base_camera'])
  packet.update(valid=True,capture_monotonic_ns=capture,point_camera_m=(bc[:3,:3].T@(skin-bc[:3,3])).tolist(),normal_out_camera=(bc[:3,:3].T@normal).tolist(),quality=dict(surface_points=100,feature_inliers=20,plane_rms_m=.001,plane_inlier_ratio=.95))
  r.publish('robot:wrist:observation:v1',json.dumps(packet))
 def cmd(action,target):
  p=envelope(target);p['action']=action;r.publish('robot:wrist:command:v1',json.dumps(p))
 def pump(seconds,target,action=None,old=False,observations=True,commands=True):
  global state
  deadline=time.monotonic()+seconds
  initial=True
  while time.monotonic()<deadline:
   if robot.poll() is not None:raise RuntimeError((folder/'robot.log').read_text())
   if observations:obs(target,old)
   if commands:cmd(action if initial and action else 'heartbeat',target)
   initial=False
   time.sleep(.035)
   for _ in range(50):
    try:m=sub.get_message(timeout=0)
    except redis.ConnectionError:break
    if m and m['type']=='message':
     state=json.loads(m['data']);history.append(state);history[:]=history[-100:]
     events.append((state['follow_state'],state.get('follow_reason','')))
    elif m is None:break
 pump(31. if args.continuous else .2,'one')  # build capture-time pose history before confirming
 pump(.6,'one','begin')
 assert any(s=='following' for s,_ in events),('never followed',events[-10:])
 pump(.15,'one',old=True)
 assert state['follow_state']=='hold',('replay not held',state)
 held=len(events);pump(.3,'one')
 assert all(s=='hold' for s,_ in events[held:]),'heartbeat auto resumed'
 pump(.5,'two','resume')
 assert any(s=='following' for s,_ in events[held:]),('resume failed',events[-10:])
 pump(.65,'two',commands=False)
 assert state['follow_state']=='hold','command timeout did not Hold'
 pump(.3,'two')
 assert state['follow_state']=='hold','heartbeat resumed after command timeout'
 pump(.3,'three','resume')
 assert state['follow_state']=='following','explicit resume after timeout failed'
 pump(.3,'three',observations=False)
 assert state['follow_state']=='hold','observation timeout did not Hold'
 pump(.3,'four','resume')
 assert state['follow_state']=='following','fresh target resume failed'
 wrong=envelope('four');wrong.update(action='heartbeat',runtime_session_id='old_robot_session')
 r.publish('robot:wrist:command:v1',json.dumps(wrong))
 pump(.2,'four')
 assert state['follow_state']=='hold','foreign Robot session not held'
 pump(.3,'five','resume')
 assert state['follow_state']=='following','session rejection resume failed'
 duplicate=envelope('five');duplicate['action']='heartbeat'
 r.publish('robot:wrist:command:v1',json.dumps(duplicate))
 r.publish('robot:wrist:command:v1',json.dumps(duplicate))
 pump(.2,'five')
 assert state['follow_state']=='hold','duplicate sequence not held'
 pump(.3,'six','resume')
 assert state['follow_state']=='following','duplicate rejection resume failed'
 r.execute_command('CLIENT','KILL','TYPE','pubsub')
 pump(2.,'six')
 assert state['follow_state']=='hold','subscription reconnect auto resumed'
 pump(.4,'seven','resume')
 assert state['follow_state']=='following','reconnect explicit resume failed'
 cmd('pause','seven');pump(.2,'seven')
 assert state['follow_state']=='hold','pause not held'
 robot.send_signal(2);robot.wait(timeout=5)
 rows=list(csv.DictReader((folder/'runtime.csv.wrist.csv').open()))
 assert rows and any(row['valid']=='1' for row in rows)
 assert any(row['planner_attempted']=='1' and row['planner_valid']=='1' for row in rows), 'planner never accepted a target'
 summary=json.loads((folder/'runtime.summary.json').read_text())
 assert summary['simulated'] and summary['mode']=='dry_run'
 assert summary['servo']['sent_sequence']==0
 if args.unlimited_excursion:
  physical=list(csv.DictReader((folder/'runtime.csv').open()))
  xyz=np.array([[float(row[k]) for k in ('actual_x_m','actual_y_m','actual_z_m')] for row in physical])
  excursion=float(np.linalg.norm(xyz-xyz[0],axis=1).max())
  assert excursion>.005, ('override never exercised beyond 5mm',excursion)
 assert summary['control']['wrist_total_excursion_limits_enabled'] == (not args.unlimited_excursion)
 assert summary['control']['wrist_candidate_trial'] == args.candidate_trial
 assert summary['control']['force_sensor_enabled'] == (not args.no_force)
 if args.no_force:
  main_rows=list(csv.DictReader((folder/'runtime.csv').open()))
  assert all(row['wrench_valid']=='0' for row in main_rows), 'fabricated valid wrench'
 print(json.dumps({'directory':str(folder),'rows':len(rows),'events':list(dict.fromkeys(events)),'result':summary['result'],'fault':summary['fault_code'],'servo_sent':summary['servo']['sent_sequence']},indent=2))
finally:
 if robot and robot.poll() is None:robot.terminate();robot.wait(timeout=5)
 server.terminate();server.wait(timeout=5)
 redis_log.close();robot_log.close()
 print('artifacts',folder)

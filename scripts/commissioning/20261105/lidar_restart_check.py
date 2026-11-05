import json,os,signal,subprocess,time,datetime,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
out=Path.home()/'dase7503-commissioning/wireless-20261105/lidar-restart-check'
out.mkdir(exist_ok=False)
rclpy.init();node=Node('wireless_lidar_restart_check')
rows=[];received=[];since=0;current=None;passed=True;error=None
def scan(m):
    stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    if stamp>=since:
        received.append((stamp,sum(math.isfinite(x) for x in m.ranges)))
sub=node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
try:
    for i in range(5):
        received.clear();since=time.time()
        with (out/f'cycle-{i+1}.log').open('w') as log:
            current=subprocess.Popen([str(Path.home()/'dase7503-commissioning/start-lidar.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+15
            while len(received)<20 and time.monotonic()<deadline and current.poll() is None:
                rclpy.spin_once(node,timeout_sec=.02)
            time.sleep(i*.017)
            os.killpg(current.pid,signal.SIGINT)
            forced=False
            try:code=current.wait(timeout=3)
            except subprocess.TimeoutExpired:
                forced=True;os.killpg(current.pid,signal.SIGTERM)
                try:code=current.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(current.pid,signal.SIGKILL);code=current.wait(timeout=2)
            try:os.killpg(current.pid,0);gone=False
            except ProcessLookupError:gone=True
            item={'cycle':i+1,'fresh_scan_count':len(received),'minimum_finite_points':min((v[1] for v in received),default=0),
              'exit_code':code,'forced_termination':forced,'group_gone':gone,
              'passed':len(received)>=20 and all(v[1]>0 for v in received) and code==0 and not forced and gone}
            rows.append(item);print(json.dumps(item),flush=True)
            current=None
            if not item['passed']:passed=False;break
except Exception as exc:
    passed=False;error=repr(exc)
finally:
    if current is not None:
        try:os.killpg(current.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:current.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(current.pid,signal.SIGKILL);current.wait(timeout=2)
    node.destroy_node();rclpy.try_shutdown()
    report={'time':datetime.datetime.now().astimezone().isoformat(),'passed':passed and len(rows)==5,'cycles':rows,'error':error,'chassis_commands_sent':False}
    (out/'report.json').write_text(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)

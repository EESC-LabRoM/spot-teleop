"""Integration check with synthetic hand samples in DDS 188; refuses a live camera.
Run in the simulation control container with ROS sourced. This moves only the simulator.
"""
import json,math,time,urllib.request
import numpy as np
import rclpy
from std_msgs.msg import String
BASE='http://127.0.0.1:8791'
def status():
    s=json.load(urllib.request.urlopen(BASE+'/state',timeout=3))
    assert s.get('simulation_only') is True and s.get('domain')==188
    return s
def control(action):
    req=urllib.request.Request(BASE+'/control',data=json.dumps({'action':action}).encode(),
        headers={'X-Sim-Control':'1','Content-Type':'application/json','Origin':BASE})
    return json.load(urllib.request.urlopen(req,timeout=3))
def wait_robot():
    deadline=time.monotonic()+180
    while time.monotonic()<deadline:
        try:
            s=status()
            if s.get('mpc_warmed') and s['sim_age_ms'] is not None and s['sim_age_ms']<300:return s
        except OSError:pass
        time.sleep(.2)
    raise AssertionError('Simulation did not start')
def main():
    initial=wait_robot()
    assert initial['overlay_age_ms'] is None or initial['overlay_age_ms']>5000,'Live operator camera: skip synthetic input'
    rclpy.init(domain_id=188);node=rclpy.create_node('fr3_synthetic_recovery_check')
    pub=node.create_publisher(String,'/fr3_sim/operator/sample',1)
    p=[0.,0.,0.];q=[0.,0.,0.,1.]
    def feed(seconds,send=True,old=False):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if send:
                ns=time.time_ns()-(5_000_000_000 if old else 0)
                d=dict(version=2,orientation_mode='full_6d',stamp_ns=ns,orientation_stamp_ns=ns,frame='fr3_link0',p=p,q=q,gesture=None,gesture_score=0.)
                pub.publish(String(data=json.dumps(d)))
            rclpy.spin_once(node,timeout_sec=0.);time.sleep(.05)
    try:
        control('stop');feed(.3);control('start');feed(.2)
        first=status();control('start');feed(.8)
        assert status()['phase']=='active',status()
        print('PASS Start, calibration and duplicate-click idempotence',flush=True)
        before=np.array(status()['tcp']);p[0]=.015;feed(2.5);moved=np.array(status()['tcp'])
        assert .004<np.linalg.norm(moved-before)<.03,(before,moved)
        print('PASS measured hand-to-robot movement (m)',(moved-before).tolist(),flush=True)
        feed(2.,send=False);paused=status()
        assert paused['phase']=='paused',paused
        held=np.array(paused['tcp']);feed(.7,old=True)
        assert status()['phase']=='paused','Stale data caused recovery'
        assert np.linalg.norm(np.array(status()['tcp'])-held)<.003,'Robot moved without fresh hand'
        print('PASS loss holds position beyond old 1.5 s cutoff; stale samples rejected',flush=True)
        p[:]=[.4,.2,-.1];q[:]=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)];feed(1.)
        recovered=status();assert recovered['phase']=='active',recovered
        assert np.linalg.norm(np.array(recovered['tcp'])-held)<.004,'Jump on reacquisition'
        print('PASS automatic recovery at new hand location, with no robot jump',flush=True)
        control('stop');held=np.array(status()['tcp']);p[0]+=.15;feed(1.)
        assert status()['phase']=='idle' and np.linalg.norm(np.array(status()['tcp'])-held)<.003
        print('PASS explicit Stop stays stopped despite subsequent fresh movement',flush=True)
    finally:
        control('stop');control('home');node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()

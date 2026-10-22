"""Exercise only the simulation HTTP session; verify measured physics responses."""
import json, math, time, urllib.request
BASE = 'http://127.0.0.1:8791'
def state():
    s = json.load(urllib.request.urlopen(BASE+'/state', timeout=3))
    assert s.get('simulation_only') is True and s.get('domain') == 188, 'Not the simulation session'
    return s
def action(name):
    state()
    req = urllib.request.Request(BASE+'/control', data=json.dumps({'action':name}).encode(),
          headers={'Content-Type':'application/json','X-Sim-Control':'1'})
    return json.load(urllib.request.urlopen(req, timeout=3))
def wait(predicate, limit=45):
    deadline = time.monotonic()+limit
    while time.monotonic()<deadline:
        s=state()
        if predicate(s):return s
        time.sleep(.3)
    raise AssertionError('Timed out: '+json.dumps(state()))
try:
    start=state();action('test');end=wait(lambda s:s['phase']=='idle')
    delta=[b-a for a,b in zip(start['tcp'],end['tcp'])]
    assert abs(delta[1]+.03)<.009 and abs(delta[0])<.009 and abs(delta[2])<.009, delta
    print('Translation delta (m):',delta,flush=True)
    start=state();action('rotate');end=wait(lambda s:s['phase']=='idle')
    dot=abs(sum(a*b for a,b in zip(start['tcp_quaternion'],end['tcp_quaternion'])))
    degrees=math.degrees(2*math.acos(min(1.,dot)))
    assert 7.8<degrees<12.2,degrees
    print('Measured rotation (deg):',round(degrees,2),flush=True)
    action('close');end=wait(lambda s:s['finger_width_m']<.003)
    print('Closed width (mm):',round(end['finger_width_m']*1000,2),flush=True)
    action('open');end=wait(lambda s:s['finger_width_m']>.077)
    print('Open width (mm):',round(end['finger_width_m']*1000,2),flush=True)
    action('home');wait(lambda s:s['phase']=='idle')
    print('Home completed',flush=True)
finally:
    action('stop')

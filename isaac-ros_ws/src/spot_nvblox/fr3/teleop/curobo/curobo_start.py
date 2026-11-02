"""Controller handover policy, called only by the operator's real Start button."""
import time

def start_reason(status):
 """Read-only preflight: standby may prepare, without requiring an active JTC."""
 if not status.get('command_ready'):return status.get('packet_reason') or 'No fresh valid MPC command'
 if status.get('controllers_age_ms',1e9)>800:return 'Controller state stale'
 if status.get('hardware_publisher_count')!=1:return 'Unexpected joint command publisher ownership'
 controllers=status.get('controllers',{})
 if controllers.get('fr3_arm_controller') not in ('active','inactive'):return 'Murilo joint controller not loaded'
 allowed={'fr3_arm_controller','cartesian_impedance_target_controller','joint_state_broadcaster','franka_robot_state_broadcaster'}
 if any(v=='active' and k not in allowed for k,v in controllers.items()):return 'Another controller is active'
 return None

def prepare(get_status,switch,arm,disarm,timeout=2.,cancelled=lambda:False):
 if cancelled():return False,'Start cancelled'
 status=get_status()
 why=start_reason(status)
 if why:return False,why
 if status.get('armed') and status.get('mode_ready'):return True,'cuRobo ready'
 controllers=status.get('controllers',{})
 old_cart=controllers.get('cartesian_impedance_target_controller')=='active';old_joint=controllers['fr3_arm_controller']=='active';changed=False
 try:
  if cancelled():raise RuntimeError('Start cancelled')
  if not old_joint or old_cart:
   if not switch(['fr3_arm_controller'],['cartesian_impedance_target_controller'] if old_cart else []):return False,'Controller handover failed'
   changed=True
  deadline=time.monotonic()+timeout
  while True:
   if cancelled():raise RuntimeError('Start cancelled')
   latest=get_status();states=latest.get('controllers',{})
   if states.get('fr3_arm_controller')=='active' and states.get('cartesian_impedance_target_controller','inactive')=='inactive':break
   if time.monotonic()>deadline:raise RuntimeError('Controller handover confirmation timed out')
   time.sleep(.05)
  if cancelled():raise RuntimeError('Start cancelled')
  ok,message=arm()
  if not ok:raise RuntimeError(message)
  if cancelled():raise RuntimeError('Start cancelled')
  return True,message
 except Exception as error:
  disarm()
  if changed:
   restored=switch(['cartesian_impedance_target_controller'] if old_cart else [],[] if old_joint else ['fr3_arm_controller'])
   if not restored:return False,str(error)+'; output disarmed, controller rollback failed'
  return False,str(error)

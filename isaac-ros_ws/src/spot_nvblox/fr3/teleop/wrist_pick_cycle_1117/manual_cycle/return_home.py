import rclpy,subprocess,json
from controller_manager_msgs.srv import SwitchController
rclpy.init();n=rclpy.create_node('wrist_user_return_home');c=n.create_client(SwitchController,'/controller_manager/switch_controller');assert c.wait_for_service(timeout_sec=15)
def switch(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=5;f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=8);assert f.done() and f.result().ok
switch(True)
try:
 p=subprocess.run(['python3','-u','/lab/fr3_zed_runtime/fr3_home_joints_effort.py']);assert p.returncode==0,'Home refused/failed'
finally:switch(False);n.destroy_node();rclpy.shutdown()
print('HOME_FINISHED_JTC_INACTIVE',flush=True)

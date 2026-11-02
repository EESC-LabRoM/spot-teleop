import sys,rclpy
from std_srvs.srv import SetBool,Trigger
rclpy.init(domain_id=int(sys.argv[1]));n=rclpy.create_node('zed_live_control_request')
type_=SetBool if len(sys.argv)>3 else Trigger
c=n.create_client(type_,sys.argv[2])
if not c.wait_for_service(timeout_sec=5):raise SystemExit('Service unavailable: '+sys.argv[2])
request=type_.Request(data=sys.argv[3]=='true') if type_==SetBool else type_.Request()
f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=8)
if not f.done() or f.result() is None:raise SystemExit('Service did not answer')
r=f.result();print(r.success,r.message)
n.destroy_node();rclpy.shutdown()
if not r.success:raise SystemExit(2)

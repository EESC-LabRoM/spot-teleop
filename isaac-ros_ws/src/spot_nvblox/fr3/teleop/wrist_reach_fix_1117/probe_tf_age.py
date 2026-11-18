import time,rclpy,tf2_ros,json
rclpy.init();n=rclpy.create_node('wrist_tf_age_probe');b=tf2_ros.Buffer();tf2_ros.TransformListener(b,n)
out=[];end=time.monotonic()+4
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.01)
 try:
  t=b.lookup_transform('fr3_link0','fr3_hand_tcp',rclpy.time.Time());out.append((n.get_clock().now().nanoseconds-t.header.stamp.sec*10**9-t.header.stamp.nanosec)*1e-9)
 except Exception:pass
print(json.dumps(dict(n=len(out),min=min(out),max=max(out),negative=sum(x<0 for x in out),stale=sum(x>=.3 for x in out))))
n.destroy_node();rclpy.shutdown()

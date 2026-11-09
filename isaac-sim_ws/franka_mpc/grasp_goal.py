# Grasp sequence publisher for curobo_mpc_node.py on the sim Franka (replaces wrist_path.py for a grasp). Reads grasps/<object>_selected.json
# (grasp_filter.py) and publishes on sim time:
#   /wrist_pose          PoseStamped, hand flange in fr3_link0, 30 Hz: retract pose -> pregrasp -> straight in along the approach axis ->
#                        (gripper closes) -> lift straight up in the base frame -> hold. Eased (min-jerk) segments, orientation slerped.
#   /gripper_width_cmd   Float32, target of each finger joint (0.04 open ... 0 closed), ramped while closing; franka_ros_arm.py applies it.
#   /grasp_target        PoseStamped, the TRUE grasp hand pose (fr3_link0), only with -p grasp_target:=true and only until the gripper
#                        starts closing: the target of the MPC node's attraction field (pf_mode:=grasp).
#   Scripted operator: -p op_err:=[dx,dy,dz,rx,ry,rz] (m, rotation vector in rad, base frame) shifts and turns where the "operator" aims the
#                        pregrasp, approach and grasp (the lift starts from there), standing in for the teleop interface's error.
#   /target_object/clear_box   only with -p clear_box:=true (OFF by default since 10-15): Marker CUBE of the target's AABB that the MPC node clears
#                        from the ESDF. Replaced by the target's mask in nvblox (target_mask.py + nvblox human_with_static_tsdf).
#   Lever turn (HiveBoard valve): -p turn_deg:=-80 -p hinge:=[cx,cy,cz,ax,ay,az] (point + axis, base frame) replaces the lift: the hand follows the lever's arc
#                        about the hinge for -p turn_s seconds (default 3.5). turn_deg 0 (default) = the lift, unchanged.
#   python grasp_goal.py --ros-args -p use_sim_time:=true -p grasp_json:=/fm/runs/<run>/grasps/drill_selected.json   (run_grasp_goal.sh)
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

P_HOME = np.array([0.1106, 0.0, 0.5907])            # hand at the retract pose (cuRobo FK, as wrist_path.py)
Q_HOME = np.array([1.0, 0.0, 0.0, 0.0])              # hand pointing down (x, y, z, w)
# seconds: approach the pregrasp (T_PRE per leg; with `via` waypoints the straight legs home -> via... -> pregrasp), settle, descend along
# the approach axis, close, wait for the grip, lift, hold
T_PRE, T_SETTLE, T_IN, T_CLOSE, T_GRIP, T_LIFT, T_HOLD = 4.0, 0.5, 2.5, 1.0, 0.8, 3.0, 1.0
LIFT = 0.15


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def slerp(q0, q1, s):
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0: q1, d = -q1, -d                        # shortest way round
    if d > 0.9995: q = q0 + s * (q1 - q0)
    else:
        th = math.acos(d); q = (math.sin((1 - s) * th) * q0 + math.sin(s * th) * q1) / math.sin(th)
    return q / np.linalg.norm(q)


def rotvec_quat(r):                                  # rotation vector -> quaternion (x, y, z, w)
    th = float(np.linalg.norm(r))
    if th < 1e-12: return np.array([0.0, 0.0, 0.0, 1.0])
    return np.array([*(np.sin(th / 2) * r / th), np.cos(th / 2)])


def quat_mul(a, b):                                  # (x, y, z, w) Hamilton product a * b
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


class GraspGoal(Node):
    def __init__(self):
        super().__init__('grasp_goal')
        g = json.load(open(self.declare_parameter('grasp_json', '').value))
        self.frame = self.declare_parameter('frame_id', 'fr3_link0').value
        self.lift = float(self.declare_parameter('lift', LIFT).value)
        self.turn = math.radians(float(self.declare_parameter('turn_deg', 0.0).value))
        hv = [float(v) for v in self.declare_parameter('hinge', [0.0] * 6).value]; self.hc, self.hax = np.array(hv[:3]), np.array(hv[3:])
        self.t_lift = float(self.declare_parameter('turn_s', 3.5).value) if self.turn else T_LIFT
        # optional hand-flange waypoints (fr3_link0) between the retract pose and the pregrasp, e.g. [0.60,0.17,0.36]: a transit that
        # crosses the scene so the MPC has to avoid what is in the way; the orientation is slerped over the whole transit
        vf = [float(v) for v in self.declare_parameter('via', [-1.0]).value]       # flat x y z ..., e.g. [0.60,0.17,0.36]; [-1.0] = none
        self.via = [np.array(vf[i:i + 3]) for i in range(0, len(vf) - len(vf) % 3, 3)] if len(vf) >= 3 else []
        self.legs = [P_HOME] + self.via
        self.p_pre, self.q_pre = np.array(g['pregrasp'][:3]), np.array(g['pregrasp'][3:])
        self.p_gr, self.q_gr = np.array(g['grasp'][:3]), np.array(g['grasp'][3:])
        self.p_true, self.q_true = self.p_gr.copy(), self.q_gr.copy()
        e = [float(v) for v in self.declare_parameter('op_err', [0.0] * 6).value]
        if any(e):
            dq = rotvec_quat(np.array(e[3:6]))
            self.p_pre, self.p_gr = self.p_pre + np.array(e[:3]), self.p_gr + np.array(e[:3])
            self.q_pre, self.q_gr = quat_mul(dq, self.q_pre), quat_mul(dq, self.q_gr)
        self.op_err = e
        self.grasp_target = bool(self.declare_parameter('grasp_target', False).value)
        self.tpub = self.create_publisher(PoseStamped, '/grasp_target', 10)
        self.box_c, self.box_h = np.array(g['target_center']), np.array(g['target_half'])
        self.obj = g['object']
        self.pub = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        self.gpub = self.create_publisher(Float32, '/gripper_width_cmd', 10)
        self.bpub = self.create_publisher(Marker, '/target_object/clear_box', 10)
        self.t0 = None
        self.done = False
        self.stage = ''
        self.t_tr = T_PRE * (1 + len(self.via))      # transit time
        self.marks = [self.t_tr, self.t_tr + T_SETTLE, self.t_tr + T_SETTLE + T_IN, self.t_tr + T_SETTLE + T_IN + T_CLOSE,
                      self.t_tr + T_SETTLE + T_IN + T_CLOSE + T_GRIP, self.t_tr + T_SETTLE + T_IN + T_CLOSE + T_GRIP + self.t_lift]
        self.t_end = self.marks[-1] + T_HOLD
        self.create_timer(1.0 / 30.0, self.tick)
        if self.declare_parameter('clear_box', False).value:   # the old coarse AABB for the MPC's ESDF clear (off: the target is masked out of the map instead)
            self.create_timer(0.5, self.send_box)
        self.get_logger().info(f'grasp {self.obj}: hand at {self.p_gr.round(3).tolist()}, pregrasp {self.p_pre.round(3).tolist()}, lift {self.lift} m, '
                               f'{self.t_end:.1f} s of sim time; operator error {self.op_err}, grasp target {self.grasp_target}; clear box centre {self.box_c.round(3).tolist()} half {self.box_h.round(3).tolist()}')

    def send_box(self):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = 'target_clear', 0, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in self.box_c)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(2 * v) for v in self.box_h)
        self.bpub.publish(m)

    def pose_at(self, t):
        t1, t2, t3, t4, t5, t6 = self.marks
        if t < t1:
            k = min(int(t / T_PRE), len(self.legs) - 1); a, b = self.legs[k], (self.via + [self.p_pre])[k]
            s = min_jerk((t - k * T_PRE) / T_PRE); q = slerp(Q_HOME, self.q_pre, (k + s) / len(self.legs))
            return a + (b - a) * s, q, 0.04, ('to waypoint %d' % (k + 1) if k < len(self.via) else 'to pregrasp')
        if t < t2: return self.p_pre, self.q_pre, 0.04, 'settle'
        if t < t3:
            s = min_jerk((t - t2) / T_IN); return self.p_pre + (self.p_gr - self.p_pre) * s, self.q_gr, 0.04, 'approach'
        if t < t4: return self.p_gr, self.q_gr, 0.04 * (1 - (t - t3) / T_CLOSE), 'close'
        if t < t5: return self.p_gr, self.q_gr, 0.0, 'grip'
        if self.turn:                                # the hand rides the lever: rigid rotation about the hinge, p' = c + R (p - c), q' = R q
            th = self.turn * min_jerk((t - t5) / self.t_lift); k = self.hax / np.linalg.norm(self.hax); r = self.p_gr - self.hc
            rr = r * math.cos(th) + np.cross(k, r) * math.sin(th) + k * float(k @ r) * (1 - math.cos(th))
            return self.hc + rr, quat_mul(rotvec_quat(k * th), self.q_gr), 0.0, ('turn' if t < t6 else 'hold')
        if t < t6:
            s = min_jerk((t - t5) / T_LIFT); return self.p_gr + np.array([0, 0, self.lift]) * s, self.q_gr, 0.0, 'lift'
        return self.p_gr + np.array([0, 0, self.lift]), self.q_gr, 0.0, 'hold'

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:                               # no /clock yet
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info(f'start at sim t={now:.3f}')
        t = now - self.t0
        p, q, w, stage = self.pose_at(t)
        if stage != self.stage:
            self.stage = stage; self.get_logger().info(f'sim t={t:.2f}: {stage}')
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in q)
        self.pub.publish(m)
        self.gpub.publish(Float32(data=float(w)))
        if self.grasp_target and t < self.marks[2]:                      # until the gripper starts closing
            g = PoseStamped(); g.header = m.header
            g.pose.position.x, g.pose.position.y, g.pose.position.z = (float(v) for v in self.p_true)
            g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = (float(v) for v in self.q_true)
            self.tpub.publish(g)
        if t >= self.t_end:
            self.get_logger().info(f'done at sim t={now:.3f} ({t:.2f} s)')
            self.done = True


def main():
    rclpy.init()
    n = GraspGoal()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

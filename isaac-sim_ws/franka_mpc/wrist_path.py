# Goal source for curobo_mpc_node.py on the sim Franka: publishes /wrist_pose (PoseStamped in the base frame, fr3_link0, 30 Hz like the
# ZED wrist detector) on sim time. Same path as franka_mpc_isaac.py --mode path (circle + vertical wave, constant speed,
# eased start and stop), raised to z 0.60 so the fingertips stay 0.2 m above every object in the workcell. The hand
# first eases from the retract pose onto the path, then exits after the final hold.
# path:=sweep (checkpoint 2, obstacle avoidance): the paper's cone experiment on the Franka. The fingertip reference sweeps
# sideways at 24 cm above the table through the blue bottle's upper part ("veja", 30 cm tall, at x 0.55 y 0.02 in the base
# frame) and back, eased (min-jerk) segments with holds, like an operator sweeping into an obstacle.
#   python wrist_path.py --ros-args -p use_sim_time:=true [-p path:=sweep]   (Isaac image Python + Jazzy rclpy: run_goal.sh)
import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

P_HOME = np.array([0.1106, 0.0, 0.5907])            # cuRobo FK of the hand at retract_config (FR3 = Panda kinematics)
CENTER, RADIUS, WAVE, PERIOD, LAPS = np.array([0.45, 0.0, 0.60]), 0.15, 0.04, 6.0, 2.0
DOWN = (1.0, 0.0, 0.0, 0.0)                          # hand pointing down (x, y, z, w)
T_APPROACH, T_SETTLE, T_RAMP, T_HOLD = 3.0, 0.5, 1.0, 1.0
W_FULL = 2 * math.pi / PERIOD
T_LOOP = LAPS * PERIOD + T_RAMP                      # with half-speed ramps the laps still total LAPS
T_END = T_APPROACH + T_SETTLE + T_LOOP + T_HOLD
TCP = 0.1034                                         # fingertip point below the hand flange (fr3_hand) with the hand pointing down
SW_X, SW_TIP_Z = 0.55, 0.24                          # defaults (params sweep_x, sweep_tip_z): fingertips 24 cm above the
                                                     # table, 6 cm below the bottle top (runs/obstacle_2zed_buf3). Lower,
                                                     # through the bottle's middle, the arm stalled: the hand is 20 cm wide
                                                     # along the sweep and the short-horizon MPPI did not find the ~9 cm
                                                     # detour behind it (runs/obstacle_2zed_axis, _offaxis). 0.28 = grazing
                                                     # the top: 2 cm into the cap, ~5 cm of lift to clear it.
SW_LEGS = [(0.28, 4.0, 1.0), (-0.10, 5.0, 1.0), (0.28, 5.0, 1.0)]   # (y to reach, seconds to get there, seconds held)
                                                     # start 14 cm clear of the bottle, end before the drill
SW_T_END = sum(d + h for _, d, h in SW_LEGS)


def path_point(th):
    return CENTER + np.array([RADIUS * math.cos(th), RADIUS * math.sin(th), WAVE * math.sin(2 * th)])


def min_jerk(s):
    s = min(max(s, 0.0), 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def omega(tl):                                       # angular speed: smooth ramp up, constant, ramp down
    if tl < 0 or tl > T_LOOP:
        return 0.0
    if tl < T_RAMP:
        return W_FULL * 0.5 * (1 - math.cos(math.pi * tl / T_RAMP))
    if tl > T_LOOP - T_RAMP:
        return W_FULL * 0.5 * (1 - math.cos(math.pi * (T_LOOP - tl) / T_RAMP))
    return W_FULL


class WristPath(Node):
    def __init__(self):
        super().__init__('wrist_path')
        self.mode = self.declare_parameter('path', 'circle').value
        self.frame = self.declare_parameter('frame_id', 'fr3_link0').value   # robot base (FR3)
        self.sw_x = float(self.declare_parameter('sweep_x', SW_X).value)
        self.sw_z = float(self.declare_parameter('sweep_tip_z', SW_TIP_Z).value) + TCP   # hand-flange height
        self.pub = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        self.t0 = None
        self.t_prev = None
        self.theta = 0.0
        self.done = False
        self.create_timer(1.0 / 30.0, self.tick)
        if self.mode == 'sweep':
            self.get_logger().info(f'path: sweep at x {self.sw_x} fingertips z {self.sw_z - TCP:.3f} through y '
                                   f'{[y for y, _, _ in SW_LEGS]}, {SW_T_END:.1f} s of sim time')
        else:
            self.get_logger().info(f'path: centre {CENTER.tolist()} r {RADIUS} wave {WAVE} period {PERIOD}s laps {LAPS}, '
                                   f'{T_END:.1f} s of sim time')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:                               # no /clock yet
            return
        if self.t0 is None:
            self.t0 = self.t_prev = now
            self.get_logger().info(f'start at sim t={now:.3f}')
        t, dt = now - self.t0, now - self.t_prev
        self.t_prev = now
        if self.mode == 'sweep':
            p, t0_, prev = P_HOME.copy(), 0.0, P_HOME.copy()
            for y, d, h in SW_LEGS:                  # eased segment to (sweep_x, y, height), then hold
                nxt = np.array([self.sw_x, y, self.sw_z])
                if t < t0_ + d:
                    p = prev + (nxt - prev) * min_jerk((t - t0_) / d); break
                p = nxt; t0_ += d + h; prev = nxt
                if t < t0_: break
            end = SW_T_END
        elif t < T_APPROACH:
            p = P_HOME + (path_point(0.0) - P_HOME) * min_jerk(t / T_APPROACH)
        elif t < T_APPROACH + T_SETTLE:
            p = path_point(0.0)
        else:
            self.theta += omega(t - T_APPROACH - T_SETTLE) * dt
            p = path_point(self.theta)
        if self.mode != 'sweep':
            end = T_END
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = DOWN
        self.pub.publish(m)
        if t >= end:
            self.get_logger().info(f'done at sim t={now:.3f} ({t:.2f} s)')
            self.done = True


def main():
    rclpy.init()
    n = WristPath()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

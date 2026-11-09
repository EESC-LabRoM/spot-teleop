#!/usr/bin/env python3
"""Patch curobo_mpc_node.py: pf_mode 'grasp' distance from the HAND, correction on the operator goal (2026-11-06).

Murilo: "calculating the distance relative to the robot's hand, but the goal has to be increased relative to the goal,
not the hand ... it needs to be an attraction, not taking over control". New params (defaults = old behaviour):
  pf_dist_from      'goal' (old: distance from the operator goal) | 'hand' (FK of the MPC's current state)
  pf_max_weight_rot  cap of the orientation weight (< 0: same as pf_max_weight)
  pf_weight_rate     max change of the weights per second (0: unlimited); ramps from 0 when the target appears
Weights w_p = pf_max_weight * s, w_r = pf_max_weight_rot * s, s = smoothstep(pf_radius_m -> pf_full_radius_m) of
d = |hand - grasp| + pf_rot_m_per_rad * angle(hand, grasp). goal = op + w_p (grasp - op), slerp(op, grasp, w_r).
Caps < 1 keep it an attraction: with the stick still the hand settles partway (regrasp/tools/pf_hand_model.py
checks there is a single equilibrium, i.e. no snapping, for the chosen radius/caps).
  python3 patch_pf_hand.py NODE.py   (idempotent; refuses if the anchors are missing)
"""
import sys

p = sys.argv[1]
s = open(p).read()
if 'pf_dist_from' in s:
    print('already patched'); sys.exit(0)


def rep(old, new):
    global s
    assert s.count(old) == 1, f'anchor not found once: {old[:60]!r}'
    s = s.replace(old, new)


rep("""        self.declare_parameter('pf_hold_after_release', True)
""", """        self.declare_parameter('pf_hold_after_release', True)
        self.declare_parameter('pf_dist_from', 'goal')       # 'goal' (operator goal, old) | 'hand' (2026-11-06)
        self.declare_parameter('pf_max_weight_rot', -1.0)    # < 0: same cap as pf_max_weight
        self.declare_parameter('pf_weight_rate', 0.0)        # max dw/dt (1/s), 0 = unlimited
""")
rep("""        self._pf_hold = bool(self.get_parameter('pf_hold_after_release').value)
""", """        self._pf_hold = bool(self.get_parameter('pf_hold_after_release').value)
        self._pf_dist_from = str(self.get_parameter('pf_dist_from').value)
        self._pf_wmax_rot = float(self.get_parameter('pf_max_weight_rot').value)
        if self._pf_wmax_rot < 0.0:
            self._pf_wmax_rot = self._pf['pf_max_weight']
        self._pf_rate = float(self.get_parameter('pf_weight_rate').value)
        self._pf_w, self._pf_t = (0.0, 0.0), None                               # rate-limited (w_pos, w_rot), last time
        self._pf.update(pf_dist_from=self._pf_dist_from, pf_max_weight_rot=self._pf_wmax_rot, pf_weight_rate=self._pf_rate)
""")
rep("""        if g is None or time.time() - rx > self._pf['pf_grasp_timeout_s']:
""", """        if g is None or time.time() - rx > self._pf['pf_grasp_timeout_s']:
            self._pf_w, self._pf_t = (0.0, 0.0), None                           # next target ramps in from zero
""")
rep("""        dot = float(np.dot(q_o, q_g))
        if dot < 0.0:
            q_g, dot = -q_g, -dot
        ang = 2.0 * math.acos(min(1.0, dot))
        d = float(np.linalg.norm(p_g - p_o)) + self._pf['pf_rot_m_per_rad'] * ang
        r1, r0 = self._pf['pf_radius_m'], self._pf['pf_full_radius_m']
        sm = min(max((r1 - d) / max(r1 - r0, 1e-6), 0.0), 1.0)
        w = self._pf['pf_max_weight'] * sm * sm * (3.0 - 2.0 * sm)
        self._last_target_dist = d
        if w > 0.0:
            if dot > 0.9995:
                q = q_o + w * (q_g - q_o)
            else:
                th = math.acos(dot)
                q = (math.sin((1.0 - w) * th) * q_o + math.sin(w * th) * q_g) / math.sin(th)
            q = q / np.linalg.norm(q)
            pose.position.copy_(self.tensor_args.to_device(p_o + w * (p_g - p_o)).view_as(pose.position))
            pose.quaternion.copy_(self.tensor_args.to_device(q).view_as(pose.quaternion))
            self._pf_delta = (w * (p_g - p_o), _qmul(q, _qconj(q_o)))
        else:
            self._pf_delta = None
        if self.step_count % 25 == 0:
            self._log_info(f'Grasp attraction w={w:.2f} d={d:.3f} (dp={np.linalg.norm(p_g - p_o):.3f} m, ang={math.degrees(ang):.1f} deg)',
                           event='target_object', step=self.step_count, target_distance=d)
""", """        dot = float(np.dot(q_o, q_g))
        if dot < 0.0:
            q_g, dot = -q_g, -dot
        if self._pf_dist_from == 'hand':                                        # distance from where the hand IS
            fk = self.mpc.rollout_fn.compute_kinematics(self.current_state)
            p_r = fk.ee_pos_seq.cpu().numpy().reshape(-1)[:3]
            q_r = fk.ee_quat_seq.cpu().numpy().reshape(-1)[:4]
            q_r = q_r / np.linalg.norm(q_r)
        else:
            p_r, q_r = p_o, q_o
        ang = 2.0 * math.acos(min(1.0, abs(float(np.dot(q_r, q_g)))))
        d = float(np.linalg.norm(p_g - p_r)) + self._pf['pf_rot_m_per_rad'] * ang
        r1, r0 = self._pf['pf_radius_m'], self._pf['pf_full_radius_m']
        sm = min(max((r1 - d) / max(r1 - r0, 1e-6), 0.0), 1.0)
        sm = sm * sm * (3.0 - 2.0 * sm)
        wt_p, wt_r = self._pf['pf_max_weight'] * sm, self._pf_wmax_rot * sm
        now = time.time()
        if self._pf_rate > 0.0:                                                 # weights cannot jump
            dw = self._pf_rate * (now - self._pf_t if self._pf_t is not None else 0.0)
            w_p0, w_r0 = self._pf_w
            w = w_p0 + min(max(wt_p - w_p0, -dw), dw)
            w_r = w_r0 + min(max(wt_r - w_r0, -dw), dw)
        else:
            w, w_r = wt_p, wt_r
        self._pf_w, self._pf_t = (w, w_r), now
        self._last_target_dist = d
        if w > 0.0 or w_r > 0.0:
            if dot > 0.9995:
                q = q_o + w_r * (q_g - q_o)
            else:
                th = math.acos(dot)
                q = (math.sin((1.0 - w_r) * th) * q_o + math.sin(w_r * th) * q_g) / math.sin(th)
            q = q / np.linalg.norm(q)
            pose.position.copy_(self.tensor_args.to_device(p_o + w * (p_g - p_o)).view_as(pose.position))
            pose.quaternion.copy_(self.tensor_args.to_device(q).view_as(pose.quaternion))
            self._pf_delta = (w * (p_g - p_o), _qmul(q, _qconj(q_o)))
        else:
            self._pf_delta = None
        if self.step_count % 25 == 0:
            self._log_info(f'Grasp attraction w={w:.2f} w_rot={w_r:.2f} d={d:.3f} from {self._pf_dist_from} '
                           f'(dp={np.linalg.norm(p_g - p_r):.3f} m, ang={math.degrees(ang):.1f} deg; '
                           f'goal->grasp {np.linalg.norm(p_g - p_o):.3f} m)',
                           event='target_object', step=self.step_count, target_distance=d)
""")
open(p, 'w').write(s)
print('patched', p)

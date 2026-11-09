#!/usr/bin/env python3
"""Plan a staged real-robot grasp test (CLI). The planner itself lives in grasp_plan_lib.py (shared with regrasp/).
  python3 grasp_plan.py OUT.json [--grasp-url URL] [--depth M] [--lift M] [--z-offset M] ...
Prints the stage report and 'PROBLEMS: none' when every check passed (grasp_test.sh greps plan.json for "problems": [])."""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grasp_plan_lib as gp  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--grasp-url', default='http://127.0.0.1:8791/grasp.json')
    ap.add_argument('--mesh-url', default='http://127.0.0.1:18089/mesh.bin')
    ap.add_argument('--z-offset', type=float, default=0.0,
                    help='add to the grasp height (m), e.g. +0.02 to stay higher on a first test')
    ap.add_argument('--map-z-bias', type=float, default=0.0,
                    help='vertical correction of the CAMERA frame (m), applied to the pick, the drill cloud and the map mesh together')
    ap.add_argument('--depth', type=float, default=0.0,
                    help='push the grasp deeper along the approach axis (m), e.g. 0.025; palm-vs-cloud checked')
    ap.add_argument('--seed-q', type=lambda t: np.array([float(x) for x in t.split(',')]), default=None,
                    help='IK seed for the grasp (7 joints, comma-separated), e.g. the current arm pose when resuming')
    ap.add_argument('--lift', type=float, default=0.10, help='S3 lift target height (m); the JTC sags ~2-3 cm under the drill')
    ap.add_argument('--vmax', type=float, default=0.10, help='joint speed cap rad/s')
    ap.add_argument('--cart-step', type=float, default=0.01,
                    help='Cartesian IK step for descent stages (m); use ~0.06 for single-waypoint stages')
    ap.add_argument('--wp-min-time', type=float, default=0.4,
                    help='floor seconds per waypoint (raise with --cart-step so single-waypoint stages stay slow)')
    a = ap.parse_args()
    g = gp.load_grasp(a.grasp_url)
    V = gp.load_mesh(a.mesh_url)
    try:
        plan = gp.plan_grasp(g, V, depth=a.depth, z_offset=a.z_offset, map_z_bias=a.map_z_bias, seed_q=a.seed_q,
                             lift=a.lift, vmax=a.vmax, cart_step=a.cart_step, wp_min_time=a.wp_min_time)
    except gp.PlanError as e:
        raise SystemExit(str(e))
    json.dump(plan, open(a.out, 'w'), indent=1)
    print(f'map_z_bias {a.map_z_bias:+.3f}  depth {a.depth:+.3f}')
    print(f'grasp hand pos {plan["grasp_pos"]}  TCP {plan["grasp_tcp"]}  approach {plan["approach"]}  (z_offset {a.z_offset:+.3f})')
    print(f'IK grasp err {plan["ik_err_mm"]} mm / {plan["ik_err_deg"]} deg; table z (map) {plan["table_z"]:.3f}')
    for k, r in plan['checks'].items():
        print(f'  {k:18s} {r}' + (f'  time {plan["stages"][k]["t"][-1]:.1f} s' if k in plan['stages'] else ''))
    print('PROBLEMS:', plan['problems'] if plan['problems'] else 'none')


if __name__ == '__main__':
    main()

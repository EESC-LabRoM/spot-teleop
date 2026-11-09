"""Export a grasp run's pick.json + cloud.npy as grasp_plan.py input.

grasp_plan.py expects {R (3x3), pick, cloud, run} (it reads via urllib, so a
file:// URL works). Run in vm-viewer:
  python3 export_plan_input.py <run_dir> <out_json>
"""
import json
import math
import sys

import numpy as np

R, O = sys.argv[1], sys.argv[2]
pick = json.load(open(f'{R}/pick.json'))
cloud = np.load(f'{R}/cloud.npy')
x, y, z, w = pick['quat_xyzw']
n = math.sqrt(x * x + y * y + z * z + w * w)
x, y, z, w = x / n, y / n, z / n, w / n
Rm = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
json.dump(dict(R=Rm, pick=pick['pos'], cloud=cloud.tolist(), run='dual2-manual'), open(O, 'w'))
print('wrote', O, 'pick', pick['pos'])

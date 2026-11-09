"""Dedicated CPU physics process; no ROS, network, planner, CUDA or hardware I/O."""
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1]/part) for part in ('remote_sim', 'teleop/curobo')]
from command_guard import ARM, LOW, HIGH


def physics_worker(seed, settings, observation, command, stopping, output):
    import mujoco
    from mujoco_remote_arm import MujocoArm, DT
    arm = MujocoArm()
    # Simulation initialization only. Never publish the seed as measured feedback.
    for index, value in enumerate(seed):
        arm.data.qpos[arm.qpos_adr[index]] = value
        arm.data.qvel[arm.dof_adr[index]] = 0.
    arm.set_targets(ARM, seed, [0.]*7)
    mujoco.mj_forward(arm.model, arm.data)
    wall0 = time.monotonic()
    due = wall0
    error = None
    max_v = max_a = 0.
    previous_v = np.zeros(7)
    previous_p, previous_q = arm.tcp_pose()
    initial_p = previous_p.copy()
    max_linear = max_angular = 0.
    entered_workspace = False
    trace = (Path(output)/'physics_trace.jsonl').open('x')
    try:
        while not stopping.is_set():
            with command.get_lock():
                incoming = list(command)
            if incoming[0] and time.monotonic()-incoming[0] < .15:
                arm.set_targets(ARM, incoming[1:8], incoming[8:15])
            else:
                arm.set_targets(ARM, arm.joint_state()[1][:7], [0.]*7)
            started = time.perf_counter()
            arm.step()
            names, q, v = arm.joint_state()
            p, quat = arm.tcp_pose()
            if not np.all(np.isfinite(q+v)) or np.any(np.array(q[:7]) < LOW) or np.any(np.array(q[:7]) > HIGH):
                raise RuntimeError('measured joint position limit/nonfinite')
            if settings:
                linear = (p-previous_p)/DT
                angular = 2*np.arccos(min(1., abs(float(np.dot(quat, previous_q)))))/DT
                max_linear = max(max_linear, float(np.linalg.norm(linear)))
                max_angular = max(max_angular, float(angular))
                if (np.linalg.norm(linear[:2]) > settings['linear_speed']*settings['speed_scale']+.001
                        or abs(linear[2]) > settings['vertical_speed']*settings['speed_scale']+.001
                        or angular > np.deg2rad(settings['angular_speed_deg'])*settings['rot_scale']+.001):
                    raise RuntimeError('measured Cartesian velocity limit')
                lo, hi = np.array(settings['workspace_min']), np.array(settings['workspace_max'])
                violation = lambda point: np.maximum(lo-point, 0)+np.maximum(point-hi, 0)
                if np.any(violation(p) > (0. if entered_workspace else violation(initial_p))+1e-5):
                    raise RuntimeError('measured workspace excursion')
                entered_workspace = entered_workspace or bool(np.all(p >= lo) and np.all(p <= hi))
            previous_p, previous_q = p.copy(), quat.copy()
            max_v = max(max_v, max(abs(x) for x in v[:7]))
            max_a = max(max_a, float(np.max(np.abs(np.array(v[:7])-previous_v)))/DT)
            previous_v = np.array(v[:7])
            for contact in arm.data.contact[:arm.data.ncon]:
                labels = [mujoco.mj_id2name(arm.model, mujoco.mjtObj.mjOBJ_BODY,
                          int(arm.model.geom_bodyid[g])) or '' for g in (contact.geom1, contact.geom2)]
                moving = [label.startswith('fr3_') and label != 'fr3_link0' for label in labels]
                robot = [label.startswith('fr3_') for label in labels]
                if (moving[0] and not robot[1]) or (moving[1] and not robot[0]):
                    raise RuntimeError('robot/environment contact: '+str(labels))
            if max_v > 1.201 or max_a > 10.05:
                raise RuntimeError(f'measured joint velocity/acceleration limit: {max_v}, {max_a}')
            with observation.get_lock():
                observation[:] = [time.time(), time.monotonic(), arm.n*DT, *q, *v, *p, *quat, 0.,
                                  time.perf_counter()-started]
            if arm.n % 4 == 0:
                trace.write(json.dumps(dict(wall_ns=time.time_ns(), sim_s=arm.n*DT,
                    q=q, v=v, tcp=p.tolist(), quat=quat.tolist()))+'\n')
            due += DT
            stopping.wait(max(0., due-time.monotonic()))
    except Exception as exception:
        error = repr(exception)
        with observation.get_lock():
            observation[28] = 1.
    finally:
        trace.close()
        (Path(output)/'physics_summary.json').write_text(json.dumps(dict(error=error,
            wall_s=time.monotonic()-wall0, sim_s=arm.n*DT, max_joint_v=max_v,
            max_joint_a=max_a, max_linear_mps=max_linear, max_angular_rps=max_angular,
            gripper='disabled', initial_q=seed), indent=2))
        arm.close()


class PhysicsProcess:
    """Brief shared-memory copies replace the former GPU/physics shared lock."""
    def __init__(self, seed, settings, output):
        values = np.asarray(seed, float)
        if values.shape != (7,) or not np.all(np.isfinite(values)) or np.any(values < LOW) or np.any(values > HIGH):
            raise ValueError('invalid/out-of-limit simulation reset joints')
        context = mp.get_context('spawn')
        self.observation = context.Array('d', 30)
        self.command = context.Array('d', 15)
        self.stopping = context.Event()
        self.process = context.Process(target=physics_worker,
            args=(seed, settings, self.observation, self.command, self.stopping, str(output)))
        self.process.start()
        deadline = time.monotonic()+15
        while not self.read()[0]:
            if not self.process.is_alive() or time.monotonic() > deadline:
                raise RuntimeError('physics process initialization failed')
            time.sleep(.01)

    def read(self):
        with self.observation.get_lock():
            values = list(self.observation)
        if values[28]:
            raise RuntimeError('physics fault; see physics_summary.json')
        return values

    @property
    def n(self):
        return round(self.read()[2]/.005)

    def joint_state(self):
        values = self.read()
        return ARM+['fr3_finger_joint1', 'fr3_finger_joint2'], values[3:12], values[12:21]

    def tcp_pose(self):
        values = self.read()
        return np.array(values[21:24]), np.array(values[24:28])

    def set_targets(self, names, q, v):
        if names != ARM or len(q) != 7 or len(v) != 7 or not np.all(np.isfinite(q+v)):
            raise ValueError('invalid simulation target')
        with self.command.get_lock():
            self.command[:] = [time.monotonic(), *q, *v]

    def close(self):
        self.stopping.set()
        self.process.join(timeout=10)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join()
            raise RuntimeError('physics shutdown timeout')

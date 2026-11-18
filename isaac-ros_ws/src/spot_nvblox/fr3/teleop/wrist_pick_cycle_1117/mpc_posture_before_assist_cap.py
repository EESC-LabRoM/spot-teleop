"""Isolated recent-posture adapter; native PF, ESDF and output guards retained."""
import ast
import time
from pathlib import Path
import mpc_entry_previous as native
import numpy as np

def _posture_write(node):
    value = node.tensor_args.to_device(node._wrist_posture_ref)
    for rollout in [o.rollout_fn for o in node.mpc.solver.optimizers] + [node.mpc.solver.safety_rollout, node.mpc.rollout_fn]:
        target = rollout.dynamics_model.retract_config
        target.copy_(value.reshape_as(target))
    target = node.goal_buffer.retract_state
    if target is not None:
        target.copy_(value.reshape_as(target))

def _posture_capture(node):
    if getattr(node, '_wrist_posture_ref', None) is None:
        node._wrist_posture_ref = node.current_state.position.detach().cpu().numpy().reshape(-1).copy()
        node._wrist_posture_time = time.monotonic()
        from motion_preference import MotionPreference
        node._wrist_motion_preference=MotionPreference()
        _posture_write(node)

def _posture_update(node, message):
    if getattr(node, '_wrist_posture_ref', None) is None:
        raise RuntimeError('Recent posture must be captured from measured state before planning')
    now = time.monotonic()
    dt = max(0., min(.05, now - node._wrist_posture_time))
    node._wrist_posture_time = now
    measured = dict(zip(message.name, message.position))
    q = np.array([measured[name] for name in node.mpc.rollout_fn.joint_names])
    if not np.all(np.isfinite(q)):
        raise ValueError('Nonfinite measured posture')
    node._wrist_posture_ref += np.clip((q-node._wrist_posture_ref)*dt/2., -np.deg2rad(5)*dt, np.deg2rad(5)*dt)
    # Retain the recent measured posture on proximal joints. A separate
    # wrist reference gives the optimizer the cheapest anatomically correct
    # direction for tool yaw; pose/world/self-collision costs still apply.
    from wrist_yaw_bias import wrist_yaw_reference
    i=node.mpc.rollout_fn.joint_names.index('fr3_joint7')
    state=node.current_state.clone()
    state.position.copy_(node.tensor_args.to_device(q).reshape_as(state.position))
    kin=node.mpc.rollout_fn.compute_kinematics(state)
    measured_q=kin.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)[0]
    goal_q=node.goal_buffer.goal_pose.quaternion.detach().cpu().numpy().reshape(-1,4)[0]
    limits=node.mpc.rollout_fn.dynamics_model.joint_limits.position.detach().cpu().numpy()
    from motion_preference import apply_preference
    goal_p=node.goal_buffer.goal_pose.position.detach().cpu().numpy().reshape(-1,3)[0]
    apply_preference(node.mpc,node._wrist_motion_preference.update(goal_p,max(dt,1e-4)))
    recent=node._wrist_posture_ref.copy()
    node._wrist_posture_ref[i]=wrist_yaw_reference(q[i],measured_q,goal_q,float(limits[0,i]),float(limits[1,i]))
    _posture_write(node)
    node._wrist_posture_ref=recent

def instrument(source, filename):
    tree = native.instrument(source, filename)
    counts = dict(capture=0, reset=0, update=0)
    class Hooks(ast.NodeTransformer):
        function = None
        def visit_ClassDef(self, node):
            return self.generic_visit(node) if node.name == 'CuroboMpcNode' else node
        def visit_FunctionDef(self, node):
            if node.name not in ('control_loop', '_set_current_state'): return node
            self.function = node.name
            return self.generic_visit(node)
        def visit_If(self, node):
            node = self.generic_visit(node)
            if self.function == 'control_loop' and ast.unparse(node.test) == '_reinit':
                counts['reset'] += 1
                node.body[:0] = ast.parse('self._wrist_posture_ref = None').body
            return node
        def visit_Expr(self, node):
            if not isinstance(node.value, ast.Call): return node
            call = ast.unparse(node.value.func)
            if self.function == '_set_current_state' and call == 'self.current_state.copy_':
                counts['capture'] += 1
                return [node, *ast.parse('_posture_capture(self)').body]
            if self.function == 'control_loop' and call == 'self.mpc.update_goal':
                counts['update'] += 1
                return [*ast.parse('_posture_update(self, _js_msg)').body, node]
            return node
    tree = Hooks().visit(tree)
    assert counts == dict(capture=1, reset=1, update=2), counts
    return ast.fix_missing_locations(tree)

def main():
    path = Path('/node/curobo_mpc_node.py')
    tree = instrument(path.read_text(), str(path))
    import sys
    sys.path.insert(0, str(path.parent))
    scope = dict(__name__='__main__', __file__=str(path), _install_guard=native._install_guard, _emit_guarded=native._emit_guarded, _posture_capture=_posture_capture, _posture_update=_posture_update)
    exec(compile(tree, str(path), 'exec'), scope)

if __name__ == '__main__': main()

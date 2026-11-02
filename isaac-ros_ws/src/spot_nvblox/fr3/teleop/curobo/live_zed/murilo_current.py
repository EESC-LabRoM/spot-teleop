"""Load Murilo's current source, adding telemetry at its native publish boundary.

The planner algorithm and trajectory construction come from /node verbatim.
Only freshness/coverage metadata is added at the private preview boundary;
the separate hardware bridge stays disarmed until a deliberate enable.
"""
import ast,hashlib,json,socket,sys
from pathlib import Path
import numpy as np

def instrument(source,filename):
 tree=ast.parse(source,filename=filename);counts=dict(init=0,publish=0,esdf=0)
 class Hooks(ast.NodeTransformer):
  def visit_ClassDef(self,node):
   if node.name!='CuroboMpcNode':return node
   return self.generic_visit(node)
  def visit_FunctionDef(self,node):
   if node.name not in ('__init__','control_loop','_on_esdf_response'):return node
   self.function=node.name;return self.generic_visit(node)
  def visit_Expr(self,node):
   value=node.value
   if not isinstance(value,ast.Call):return node
   call=ast.unparse(value.func)
   if getattr(self,'function',None)=='__init__' and call=='super().__init__':
    counts['init']+=1;return [node,*ast.parse('_install_guard(self)').body]
   if getattr(self,'function',None)=='control_loop' and call=='self.cmd_pub.publish':
    if ast.unparse(value.args[0])!='joint_cmd':raise ValueError('Unexpected publish contract')
    counts['publish']+=1
    return [*ast.parse('_emit_guarded(self, _js_msg, _pose_msg, is_feasible, ordered_names, cmd_state, pos_list, vel_list, mpc_result)').body,node]
   if getattr(self,'function',None)=='_on_esdf_response' and call.endswith('.update_voxel_data'):
    counts['esdf']+=1;return [node,*ast.parse('self.safe_esdf_ns=self.get_clock().now().nanoseconds').body]
   return node
 tree=Hooks().visit(tree)
 if counts!={'init':1,'publish':1,'esdf':1}:raise ValueError('Murilo source interface changed; refusing unsafe integration: '+str(counts))
 return ast.fix_missing_locations(tree)

def _install_guard(node):
 node.safe_port=int(node.declare_parameter('safe_packet_port',0).value)
 if node.safe_port<=0:raise ValueError('A private guarded packet port is required')
 node.safe_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
 node.safe_esdf_ns=0;node.safe_depth_ns=0
 node.depth_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);node.depth_socket.bind(('127.0.0.1',47929));node.depth_socket.setblocking(False)

def _emit_guarded(node,joints,goal,feasible,names,state,positions,velocities,result=None):
 if node.cmd_pub.topic_name!='/fr3_joystick_mpc/joint_preview':raise ValueError('Native output must remain a private preview')
 for _ in range(32):
  try:
   health=json.loads(node.depth_socket.recv(2048));ns=int(health['stamp_ns']);age=(node.get_clock().now().nanoseconds-ns)*1e-9
   if health.get('kind')=='segmented_depth' and 0<=age<.7:node.safe_depth_ns=max(node.safe_depth_ns,ns)
  except BlockingIOError:break
  except (ValueError,KeyError,TypeError):continue
 coverage=False
 try:
  spheres=node.mpc.rollout_fn.compute_kinematics(state).robot_spheres.detach().cpu().numpy().reshape(-1,4);spheres=spheres[spheres[:,3]>0]
  lo=np.asarray(node._last_grid_bb_min);hi=np.asarray(node._last_grid_bb_max)
  coverage=bool(len(spheres) and np.all(spheres[:,:3]-spheres[:,3:4]>=lo) and np.all(spheres[:,:3]+spheres[:,3:4]<=hi))
 except Exception:pass
 def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec if m is not None else 0
 packet=dict(version=1,frame='fr3_link0',stamp_ns=node.get_clock().now().nanoseconds,goal_stamp_ns=stamp(goal),joint_stamp_ns=stamp(joints),esdf_stamp_ns=node.safe_esdf_ns,depth_stamp_ns=node.safe_depth_ns,feasible=bool(feasible),grid_covers_robot=coverage,names=names,q=positions,v=velocities)
 node.safe_socket.sendto(json.dumps(packet,allow_nan=False).encode(),('127.0.0.1',node.safe_port))
 if not feasible and packet['stamp_ns']-getattr(node,'safe_debug_ns',0)>5_000_000_000:
  node.safe_debug_ns=packet['stamp_ns']
  try:
   distances=node._get_sphere_distances();kin=node.mpc.rollout_fn.kinematics;cfg=kin.kinematics_config
   actual=kin.get_state(node.current_state.position).link_spheres_tensor.detach().cpu().numpy().reshape(-1,4)
   index=cfg.link_sphere_idx_map.detach().cpu().numpy().reshape(-1);names_by_idx={v:k for k,v in cfg.link_name_to_idx_map.items()}
   debug=dict(stamp_ns=packet['stamp_ns'],depth_age_s=(packet['stamp_ns']-node.safe_depth_ns)*1e-9,spheres=[dict(index=i,link=names_by_idx.get(int(index[i])),xyzw=actual[i].tolist(),distance=float(d)) for i,d in enumerate(distances)])
   if result is not None and result.metrics.state is not None:
    rr=node.mpc.solver.safety_rollout;ss=result.metrics.state
    to_list=lambda value:value.detach().cpu().numpy().reshape(-1).tolist()
    debug['constraints']=dict(bound=to_list(rr.bound_constraint.forward(ss.state_seq)),world=to_list(rr.primitive_collision_constraint.forward(ss.robot_spheres,env_query_idx=rr._goal_buffer.batch_world_idx)),self_collision=to_list(rr.robot_self_collision_constraint.forward(ss.robot_spheres)))
    debug['state_velocity']=to_list(ss.state_seq.velocity);debug['state_acceleration']=to_list(ss.state_seq.acceleration)
   Path('/out/collision_snapshot.json').write_text(json.dumps(debug,indent=2))
  except Exception as error:Path('/out/collision_snapshot_error.txt').write_text(str(error))

 return bool(feasible)

def main():
 path=Path('/node/curobo_mpc_node.py');source=path.read_text();tree=instrument(source,str(path));digest=hashlib.sha256(source.encode()).hexdigest()
 provenance=dict(source=str(path),sha256=digest,source_mtime=path.stat().st_mtime,adapter='murilo_current.py',output='native preview + guarded metadata; hardware bridge initially disarmed')
 Path('/out/murilo_source.json').write_text(json.dumps(provenance,indent=2));print('MURILO_SOURCE '+json.dumps(provenance),flush=True)
 sys.path.insert(0,str(path.parent));ns=dict(__name__='__main__',__file__=str(path),_install_guard=_install_guard,_emit_guarded=_emit_guarded)
 exec(compile(tree,str(path),'exec'),ns)
if __name__=='__main__':main()

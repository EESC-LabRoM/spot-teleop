import sys, numpy as np, torch, json
sys.argv = ["x", "/cs/poses_map_v2.json", "/cs/world/esdf.npz", "/cs/world/segcloud.npz", "/cs/dbg", "--slab-margin", "0.016", "--margin", "0.075"]
src = open("/cs/calib_legs_map.py").read()
cut = src.index("seq = [QH] + P + [QH]")     # build world + MotionGen exactly like the planner, stop before planning
exec(compile(src[:cut], "calib_legs_map_head", "exec"))
D = np.loadtxt("/cs/sweep_map_v3.csv", delimiter=",", skiprows=1); Q = D[:, 1:8]
kin = load_yaml(a.cfg)["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0; model = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
def spheres(Qs):
    q = torch.tensor(Qs, device=tdev.device, dtype=tdev.dtype); S9 = model.get_state(q).link_spheres_tensor.cpu().numpy(); S9 = S9[:, S9[0, :, 3] > 0]; C, r = S9[..., :3], S9[0, :, 3]; st = C.std(0).max(-1) < 0.005; return C[:, ~st], r[~st]
idxs = np.arange(0, len(Q), 3); C, r = spheres(Q[idxs])
def esdf_at(Cc):
    idx = (Cc - ORG) / VOX - 0.5; d = map_coordinates(Ef, idx.reshape(-1, 3).T, order=1, mode="nearest").reshape(Cc.shape[:-1]); out = ~np.all((idx >= 0) & (idx <= np.array(E.shape) - 1), axis=-1); return np.where(out, 1.0, d) - 0.5 * VOX
gm = (esdf_at(C) - r).min(1); order = np.argsort(gm)[:6]
print("worst samples by my map check (sphere surface to map surface, buffer 0):")
for o in order:
    i = idxs[o]; q = Q[i]; si = int((esdf_at(C[o:o+1])[0] - r).argmin())
    jsq = JointState.from_position(torch.tensor(np.r_[q][None], device=tdev.device, dtype=tdev.dtype))
    ok, status = mg.check_start_state(jsq)
    print(f"  t={D[i,0]:6.1f} s  my gap {100*gm[o]:6.1f} cm at sphere {si} centre {np.round(C[o,si],3).tolist()} | cuRobo check_start_state: valid={bool(ok)} {status}")
print("--- after mg.world_coll_checker.update_voxel_data(grid) (what the MPC node does every ESDF update) ---")
mg.world_coll_checker.update_voxel_data(grid)
for o in order:
    i = idxs[o]; q = Q[i]; jsq = JointState.from_position(torch.tensor(np.r_[q][None], device=tdev.device, dtype=tdev.dtype)); ok, status = mg.check_start_state(jsq)
    print(f"  t={D[i,0]:6.1f} s  my gap {100*gm[o]:6.1f} cm | cuRobo valid={bool(ok)} {status}")
# is cuRobo's voxel data what I think? query a handful of points through a tiny sphere distance
wc = mg.world_coll_checker; print("collision checker:", type(wc).__name__, "| voxel grid names:", [g.name for g in wc.world_model.voxel] if hasattr(wc, "world_model") and wc.world_model is not None else "?")

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from real_tf_poll import assemble, quat_to_R  # noqa: E402

NOW = 1794015200.0
TF = [0.579, -0.249, 0.249]
TGT = {"pos": [0.5037, -0.0957, 0.276], "quat": [0.0, 0.0, 0.0, 1.0]}
RUN_PAYLOAD = {"run": "20261107_013427_drill_loop_regen",
               "cloud": [[0.53 + 0.001 * i, -0.17, 0.27] for i in range(3309)],
               "live": {"age_s": 12.0, "held": False}}
LIVE_PAYLOAD = {"run": "g7_03@live1", "cloud": [[0.1 * i, 0.0, 0.0] for i in range(100)],
                "live": {"cloud": "live_mask_depth+zed2_mask", "n_zed2": 40, "cloud_age_s": 1.2}}
BINDING = {"run_id": "20261106_223704_drill_loop_regen", "bound_at": NOW - 900.0}


def test_quat_identity_is_I():
    R = quat_to_R([0, 0, 0, 1])
    assert all(abs(R[i][j] - (1.0 if i == j else 0.0)) < 1e-9 for i in range(3) for j in range(3)), R


def test_quat_z90():
    R = quat_to_R([0, 0, 0.7071068, 0.7071068])  # +90 deg about Z: x -> y
    assert abs(R[0][1] - -1.0) < 1e-4 and abs(R[1][0] - 1.0) < 1e-4 and abs(R[2][2] - 1.0) < 1e-9, R


def test_run_shape_payload():
    live, pts = assemble(TF, NOW - 0.2, TGT, NOW - 0.1, RUN_PAYLOAD, BINDING, {"state": "TRACKING"}, NOW)
    assert live["status"] == "ok" and live["n_points"] == 3309 and len(pts) == 3309
    assert live["pick"]["pos"] == [0.5037, -0.0957, 0.276] and live["pick"]["conf"] is None
    assert live["pick"]["R"][0][0] == 1.0 and live["tf_age_s"] == 0.2 and live["target_age_s"] == 0.1
    assert live["cloud_run"] == "20261107_013427_drill_loop_regen" and live["cloud_age_s"] == 12.0
    assert live["binding_run"] == "20261106_223704_drill_loop_regen" and live["binding_age_s"] == 900.0
    assert live["loop"] == {"state": "TRACKING"}


def test_live_shape_payload():
    live, pts = assemble(TF, NOW - 0.3, TGT, NOW - 0.2, LIVE_PAYLOAD, BINDING, None, NOW)
    assert live["n_points"] == 100 and len(pts) == 100 and live["cloud_age_s"] == 1.2
    assert live["loop"] is None


def test_missing_everything_is_empty_not_crash():
    live, pts = assemble(None, None, None, None, {"no": "cloud"}, None, None, NOW)
    assert live["status"] == "empty" and pts is None and live["n_points"] == 0
    assert live["pick"] is None and live["tf"] is None


def test_cloud_stride_caps_points():
    big = {"run": "x", "cloud": [[float(i), 0.0, 0.0] for i in range(9000)], "live": {}}
    live, pts = assemble(TF, NOW, TGT, NOW, big, None, None, NOW)
    assert live["n_points"] == 9000 and len(pts) == 4500  # stride 2 over MAX_PTS=4000
    assert pts[0][0] == 0.0 and pts[1][0] == 2.0

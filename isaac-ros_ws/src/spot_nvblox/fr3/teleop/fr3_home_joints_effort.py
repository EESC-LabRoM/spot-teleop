#!/usr/bin/env python3
"""Joint-space home via the EFFORT JointTrajectoryController (universal default).

Runs /lab/fr3_home_jtc.py as a child with the effort controller baked in, so the
pad (hold X), the dashboard (/teleop/home) and the CLI all send a true joint
home with no cuRobo, no GPU, and no driver restart (effort -> effort swap, the
impedance controller is restored afterwards). Extra CLI args (--dry-run, --pose
a,b,c,d,e,f,g, ...) are passed through. SIGINT/SIGTERM (Back on the pad) is
forwarded so the child aborts, holds, and restores impedance, same contract as
the deprecated FK fallback fr3_home_safe.py (hand-only, elbow uncontrolled).
"""
import os
from pathlib import Path
import signal
import subprocess
import sys

JTC_SCRIPT = "/lab/fr3_home_jtc.py"
CONTROLLER = "fr3_arm_controller"
PARAMS = "/lab/fr3_arm_jtc.yaml"
SPEED = "0.1"


def build_cmd(extra):
    """argv for the child: effort-JTC defaults plus caller passthrough."""
    return [sys.executable, JTC_SCRIPT,
            "--position-controller", CONTROLLER,
            "--params", PARAMS,
            "--speed-fraction", SPEED] + [str(a) for a in extra]


def main(argv):
    # Keep discovery and service replies off broken shared-memory ports and use
    # an image-sized receive buffer even when launched outside the follower.
    profile = Path(__file__).with_name('live_transport.xml')
    env = os.environ.copy()
    if profile.is_file():
        env.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile))
    proc = subprocess.Popen(build_cmd(argv), env=env)

    def _fwd(signum, frame):
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

    old = {s: signal.signal(s, _fwd) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        rc = proc.wait()
    finally:
        for s, h in old.items():
            signal.signal(s, h)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

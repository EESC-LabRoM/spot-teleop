# clone franka-seg-zed1 -> franka-seg-zed2 (same image/GPU/mounts/env/flags, zed1->zed2 in the topics only)
import json, subprocess, shlex, sys
i = json.loads(subprocess.check_output(["docker", "inspect", "franka-seg-zed1"]))[0]
hc, cf = i["HostConfig"], i["Config"]
gpu = hc["DeviceRequests"][0]["DeviceIDs"]; assert gpu == ["0"], gpu
cmd = ["docker", "run", "-d", "--rm", "--name", "franka-seg-zed2", "--gpus", "device=" + ",".join(gpu), "--network", hc["NetworkMode"], "--ipc", hc["IpcMode"], "--security-opt", "label=disable", "--entrypoint", cf["Entrypoint"][0]]
for e in cf["Env"]:
    if not e.startswith(("PATH=", "HOSTNAME=", "HOME=")): cmd += ["-e", e]
for m in i["Mounts"]: cmd += ["-v", f'{m["Source"]}:{m["Destination"]}' + (":ro" if not m["RW"] else "")]
cmd += [cf["Image"]] + [a.replace("zed1", "zed2") for a in cf["Cmd"]]
print(" ".join(shlex.quote(c) for c in cmd[:14]), "...", " ".join(shlex.quote(c) for c in cmd[-16:]), file=sys.stderr)
if "--go" in sys.argv:
    out = subprocess.run(cmd, capture_output=True, text=True); print("docker run ->", out.stdout.strip()[:20], out.stderr.strip()[:300])

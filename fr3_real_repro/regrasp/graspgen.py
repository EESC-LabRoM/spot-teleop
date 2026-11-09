"""GraspGen client (VM side): persistent server first, one-shot fallback.

Protocol (file-drop over scp; mirrored in server/graspgen_server.py, which
is standalone on the lab box and MUST keep the same names):
  client: scp cloud_<id>.npy + cloud_<id>.json {num} -> <RDIR>/inbox/
          (atomic: .tmp + rename; the server ignores *.tmp)
  server: loads the model once, polls inbox, runs inference,
          writes outbox/grasps_<id>.npy atomically + heartbeat file
  client: polls outbox (default 120 s), scp's the result back.
Server liveness = heartbeat file younger than 30 s. Anything missing,
stale or timed out falls back to the one-shot ssh path (today's recipe:
fresh python per call, model reloaded, ~2-35 s). The loop degrades to slow
runs; it never dies on the server.
"""
import json
import os
import subprocess
import time
import uuid

LAB = 'murilo@100.88.134.127'
KEYS = ['/ssh/id_lab', os.path.expanduser('~/.ssh/id_ed25519_lab_claude')]
SERVER_DIR = '/tmp/regrasp_graspgen'
HEARTBEAT = 'alive.json'
HEARTBEAT_MAX_S = 30.0
POLL_S = 1.0
TIMEOUT_S = 120.0
VENV_PY = '/home/murilo/graspgenx/.venv/bin/python'
CUDA = '1'


def new_job_id():
    return uuid.uuid4().hex[:12]


def inbox_name(jid):
    return f'cloud_{jid}.npy'


def inbox_meta_name(jid):
    return f'cloud_{jid}.json'


def result_name(jid):
    return f'grasps_{jid}.npy'


class LabTransport:
    """Real ssh/scp transport to the lab box."""

    def __init__(self, lab=LAB, keys=None, timeout=60):
        self.lab = lab
        self.timeout = timeout
        self.key = next((k for k in (keys or KEYS) if os.path.exists(k)), None)
        if self.key is None:
            raise FileNotFoundError(f'no lab key among {keys or KEYS}')

    def run(self, cmd, timeout=300):
        return subprocess.run(
            ['ssh', '-i', self.key, '-o', 'IdentitiesOnly=yes',
             '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=accept-new', self.lab, cmd],
            capture_output=True, text=True, timeout=timeout)

    def put(self, local, remote):
        return subprocess.run(
            ['scp', '-i', self.key, '-o', 'IdentitiesOnly=yes',
             '-o', 'BatchMode=yes', local, f'{self.lab}:{remote}'],
            capture_output=True, text=True, timeout=self.timeout)

    def get(self, remote, local):
        return subprocess.run(
            ['scp', '-i', self.key, '-o', 'IdentitiesOnly=yes',
             '-o', 'BatchMode=yes', f'{self.lab}:{remote}', local],
            capture_output=True, text=True, timeout=self.timeout)


class DirTransport:
    """Local-directory stand-in for the lab box (tests only)."""

    def __init__(self, root):
        self.root = root
        os.makedirs(root, exist_ok=True)
        self.cmds = []

    def _p(self, remote):
        return os.path.join(self.root, remote.lstrip('/'))

    def run(self, cmd, timeout=300):
        self.cmds.append(cmd)
        for part in cmd.split('&&'):  # emulate the exact commands used
            part = part.strip()
            if part.startswith('mkdir -p '):
                for d in part[len('mkdir -p '):].split():
                    os.makedirs(self._p(d), exist_ok=True)
            elif part.startswith('mv '):
                a, b = part[len('mv '):].split()
                os.rename(self._p(a), self._p(b))

        class _R:
            returncode = 0
            stdout, stderr = '', ''
        return _R()

    def put(self, local, remote):
        dst = self._p(remote)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(local, 'rb') as f, open(dst, 'wb') as g:
            g.write(f.read())

        class _R:
            returncode = 0
            stdout, stderr = '', ''
        return _R()

    def get(self, remote, local):
        with open(self._p(remote), 'rb') as f, open(local, 'wb') as g:
            g.write(f.read())

        class _R:
            returncode = 0
            stdout, stderr = '', ''
        return _R()

    def exists(self, remote):
        return os.path.exists(self._p(remote))


def server_alive(transport, rdir=SERVER_DIR, max_age=HEARTBEAT_MAX_S):
    """Heartbeat check. LabTransport: ssh test; DirTransport: local file."""
    if isinstance(transport, DirTransport):
        p = os.path.join(transport.root, rdir.lstrip('/'), HEARTBEAT)
        try:
            hb = json.load(open(p))
        except (OSError, ValueError):
            return False
        return time.time() - float(hb.get('wall', 0)) < max_age
    p = subprocess.run(
        ['ssh', '-i', transport.key, '-o', 'IdentitiesOnly=yes',
         '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=accept-new', transport.lab,
         f'cat {rdir}/{HEARTBEAT} 2>/dev/null'],
        capture_output=True, text=True, timeout=15)
    try:
        return time.time() - float(json.loads(p.stdout).get('wall', 0)) < max_age
    except (ValueError, KeyError):
        return False


def infer_via_server(cloud_npy, num, transport, rdir=SERVER_DIR,
                     timeout=TIMEOUT_S):
    """Submit to the persistent server, poll, collect. Returns local path."""
    jid = new_job_id()
    transport.run(f'mkdir -p {rdir}/inbox {rdir}/outbox')
    transport.put(cloud_npy, f'{rdir}/inbox/{inbox_name(jid)}.tmp')
    meta = f'/tmp/regrasp_meta_{jid}.json'
    json.dump({'num': num}, open(meta, 'w'))
    transport.put(meta, f'{rdir}/inbox/{inbox_meta_name(jid)}.tmp')
    os.remove(meta)
    transport.run(f'mv {rdir}/inbox/{inbox_name(jid)}.tmp '
                  f'{rdir}/inbox/{inbox_name(jid)} && '
                  f'mv {rdir}/inbox/{inbox_meta_name(jid)}.tmp '
                  f'{rdir}/inbox/{inbox_meta_name(jid)}')
    out = f'/tmp/regrasp_grasps_{jid}.npy'
    t0 = time.time()
    while time.time() - t0 < timeout:
        if isinstance(transport, DirTransport):
            ready = transport.exists(f'{rdir}/outbox/{result_name(jid)}')
        else:
            r = transport.run(f'test -f {rdir}/outbox/{result_name(jid)}')
            ready = r.returncode == 0
        if ready:
            transport.get(f'{rdir}/outbox/{result_name(jid)}', out)
            return out
        time.sleep(POLL_S)
    raise TimeoutError(f'graspgen server gave nothing in {timeout:.0f}s')


def infer_oneshot(cloud_npy, num, transport, rdir='/tmp/regrasp_oneshot'):
    """Today's recipe: fresh interpreter per call (model reloaded)."""
    here = os.path.dirname(os.path.abspath(__file__))
    infer_py = os.path.join(os.path.dirname(here), 'grasp_loop',
                            'grasp_infer.py')
    transport.run(f'mkdir -p {rdir}')
    transport.put(cloud_npy, f'{rdir}/cloud.npy')
    transport.put(infer_py, f'{rdir}/grasp_infer.py')
    r = transport.run(
        f'cd {rdir} && CUDA_VISIBLE_DEVICES={CUDA} timeout 600 '
        f'{VENV_PY} grasp_infer.py cloud.npy grasps.npy --num {num}',
        timeout=660)
    if r.returncode != 0:
        raise RuntimeError(f'oneshot infer failed: {r.stderr[:300]}')
    out = f'/tmp/regrasp_grasps_oneshot_{new_job_id()}.npy'
    transport.get(f'{rdir}/grasps.npy', out)
    return out


def infer(cloud_npy, num, transport, rdir=SERVER_DIR, timeout=TIMEOUT_S,
          allow_fallback=True):
    """Server when alive, one-shot fallback otherwise. Returns local path."""
    if server_alive(transport, rdir):
        try:
            return infer_via_server(cloud_npy, num, transport, rdir, timeout)
        except (TimeoutError, OSError):
            if not allow_fallback:
                raise
    return infer_oneshot(cloud_npy, num, transport)

"""Local, seekable video editor for operator bags and existing simulation clips."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import mimetypes
from pathlib import Path
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit, parse_qs
from urllib.request import Request, urlopen
import uuid

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
MEDIA = REPO/'clips_out/editor_media'
OUTPUT = REPO/'clips_out/manual_clips'


def validate_cut(source, start, end):
    if isinstance(start, bool) or isinstance(end, bool):
        raise ValueError('Informe início e fim em segundos')
    start, end = float(start), float(end)
    if not all(math.isfinite(v) for v in (start, end)) or not 0 <= start < end <= source['duration']+.001:
        raise ValueError('O início precisa vir antes do fim, dentro do vídeo')
    if end-start < .08:
        raise ValueError('Escolha ao menos 0,08 s')
    return round(start, 3), round(end, 3)


def byte_range(value, size):
    if not value:
        return 0, size-1, False
    match = re.fullmatch(r'bytes=(\d*)-(\d*)', value.strip())
    if not match or not any(match.groups()):
        raise ValueError('Invalid byte range')
    first, last = match.groups()
    if not first:
        amount = int(last)
        if amount <= 0:
            raise ValueError('Invalid suffix range')
        start, end = max(0, size-amount), size-1
    else:
        start, end = int(first), min(int(last), size-1) if last else size-1
    if not 0 <= start <= end < size:
        raise ValueError('Unsatisfiable byte range')
    return start, end, True


def atomic_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


class Editor:
    def __init__(self):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='video-export')
        self.sources = {s['id']: s for s in json.loads((MEDIA/'library.json').read_text())}
        self.path = OUTPUT/'selections.json'
        self.clips = json.loads(self.path.read_text()) if self.path.exists() else []
        self.jobs = {}
        for file in OUTPUT.glob('*/job.json'):
            job = json.loads(file.read_text())
            self.jobs[job['id']] = job
        self.simulation_busy = False
        self.demo_path = OUTPUT/'demonstrations.json'
        self.demonstrations = json.loads(self.demo_path.read_text()) if self.demo_path.exists() else []

    def public_source(self, source):
        result = {k: v for k, v in source.items() if k not in ('path', 'thumbnail')}
        result['url'] = '/media/'+source['id']
        result['thumbnail_url'] = '/thumbnail/'+source['id']
        return result

    def library(self):
        with self.lock:
            # Offline reprocessing adds sources atomically while the editor runs.
            self.sources = {s['id']: s for s in json.loads((MEDIA/'library.json').read_text())}
            return dict(sources=[self.public_source(s) for s in self.sources.values()], clips=self.clips.copy(),
                        jobs=list(self.jobs.values()), simulation_busy=self.simulation_busy,
                        demonstrations=self.demonstrations)

    def robot_state(self):
        with urlopen('http://127.0.0.1:8791/state', timeout=2) as response:
            state = json.load(response)
        if state.get('simulation_only') is not True or state.get('domain') != 188:
            raise ValueError('O controle manual exige a sessão de simulação')
        return state

    def robot_control(self, data):
        if self.simulation_busy:
            raise ValueError('Aguarde o trecho automático terminar')
        action=data['action']
        if action not in ('manual_begin','manual_home','manual_nudge','stop','open','close'):
            raise ValueError('Comando manual desconhecido')
        self.robot_state()
        request=Request('http://127.0.0.1:8791/control',data=json.dumps(dict(action=action,value=data.get('value'))).encode(),
                        headers={'Content-Type':'application/json','X-Sim-Control':'1'})
        try:
            with urlopen(request,timeout=3) as response:return json.load(response)
        except Exception as error:
            if hasattr(error,'read'):
                details=json.loads(error.read())
                raise ValueError(details.get('error','Comando recusado')) from error
            raise

    def review(self,dataset='realsense'):
        path=REPO/('clips_out/zed_protocol_analysis/simulation_review.json' if dataset=='zed' else 'clips_out/realsense_analysis/cartesian_review.json')
        feedback=OUTPUT/'review_feedback.json'
        data=json.loads(path.read_text()) if path.exists() else dict(status='preparing',runs=[])
        data['feedback']=json.loads(feedback.read_text()) if feedback.exists() else {}
        return data

    def review_feedback(self,data):
        source=data['source_id'];status=data['status'];note=str(data.get('note','')).strip()[:2000]
        if source not in self.sources or status not in ('correct','wrong','pending'):raise ValueError('Avaliação inválida')
        with self.lock:
            path=OUTPUT/'review_feedback.json';feedback=json.loads(path.read_text()) if path.exists() else {}
            current=next((r for r in self.review('zed' if source.startswith('zed_protocol_') else 'realsense')['runs'] if r['source_id']==source),None)
            if current and data.get('job_id')!=current.get('job_id'):raise ValueError('O vídeo foi atualizado. Avalie a versão nova.')
            feedback[source]=dict(status=status,note=note,saved_at=time.time(),job_id=data.get('job_id'))
            atomic_json(path,feedback)
            return dict(ok=True,feedback=feedback[source])

    def load_source(self,source_id):
        with self.lock:
            if self.simulation_busy:raise ValueError('Aguarde o trecho automático terminar')
            source=self.sources[source_id]
            if not source.get('bag'):raise ValueError('Esta gravação não tem bag de movimentos')
            self.robot_state()
            def post(port,payload,header):
                request=Request(f'http://127.0.0.1:{port}/control',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json',header:'1'})
                try:
                    with urlopen(request,timeout=5) as response:return json.load(response)
                except Exception as error:
                    if hasattr(error,'read'):raise ValueError(error.read().decode()) from error
                    raise
            post(8791,dict(action='stop'),'X-Sim-Control')
            post(8792,dict(action='load',bag='/home/spot-teleop/'+source['bag']),'X-Bag-Control')
            return dict(ok=True,source_id=source_id,label=source['label'])

    def capture_pose(self, data):
        source=self.sources[data['source_id']]
        stamp=float(data['video_time_s'])
        if not math.isfinite(stamp) or not 0<=stamp<=source['duration']+.001:
            raise ValueError('Instante inválido do vídeo')
        state=self.robot_state()
        if state.get('phase') not in ('manual','idle') or state.get('sim_age_ms') is None or state['sim_age_ms']>500:
            raise ValueError('Pare o robô na pose desejada e aguarde o estado da simulação')
        if not state.get('joint_positions') or not state.get('tcp'):
            raise ValueError('O controlador ainda está carregando as poses; aguarde')
        pose=dict(id=uuid.uuid4().hex[:12],video_time_s=round(stamp,3),saved_at=time.time(),
                  tcp_p=state['tcp'],tcp_q=state['tcp_quaternion'],joints=state['joint_positions'],
                  gripper_width_m=state.get('finger_width_m'),target=state.get('manual_target'),
                  measured_age_ms=state['sim_age_ms'])
        if source.get('bag'):pose['bag_time_s']=round(stamp+source.get('bag_offset_s',0),3)
        with self.lock:
            demo=next((d for d in self.demonstrations if d['source_id']==source['id']),None)
            if demo is None:
                demo=dict(id=uuid.uuid4().hex[:12],source_id=source['id'],source_label=source['label'],bag=source.get('bag'),
                          simulation_only=True,keyframes=[])
                self.demonstrations.append(demo)
            matching=next((p for p in demo['keyframes'] if abs(p['video_time_s']-stamp)<.5/source.get('fps',25)),None)
            if matching:
                pose['id']=matching['id'];demo['keyframes'][demo['keyframes'].index(matching)]=pose
            else:demo['keyframes'].append(pose)
            demo['keyframes'].sort(key=lambda p:p['video_time_s'])
            atomic_json(self.demo_path,self.demonstrations)
            return dict(demonstration=demo,pose=pose)

    def remove_pose(self,data):
        with self.lock:
            demo=next(d for d in self.demonstrations if d['id']==data['demo_id'])
            demo['keyframes']=[p for p in demo['keyframes'] if p['id']!=data['pose_id']]
            atomic_json(self.demo_path,self.demonstrations)
            return dict(ok=True)

    def save(self, data):
        source = self.sources[data['source_id']]
        start, end = validate_cut(source, data['start_s'], data['end_s'])
        name = str(data.get('label', '')).strip()[:120] or f'Trecho {start:.2f}–{end:.2f}'
        with self.lock:
            existing = next((c for c in self.clips if c['id'] == data.get('id')), None)
            clip = dict(id=existing['id'] if existing else uuid.uuid4().hex[:12], source_id=source['id'],
                        label=name, start_s=start, end_s=end, duration=round(end-start, 3),
                        bag=source.get('bag'), created_at=time.time())
            if source.get('bag'):
                clip['bag_start_s'] = round(start+source.get('bag_offset_s', 0), 3)
                clip['bag_end_s'] = round(end+source.get('bag_offset_s', 0), 3)
            if existing:
                self.clips[self.clips.index(existing)] = clip
            else:
                self.clips.append(clip)
            atomic_json(self.path, self.clips)
            return clip

    def remove(self, clip_id):
        with self.lock:
            self.clips = [c for c in self.clips if c['id'] != clip_id]
            atomic_json(self.path, self.clips)

    def export(self, clip_id):
        with self.lock:
            clip = next(c.copy() for c in self.clips if c['id'] == clip_id)
            job_id = uuid.uuid4().hex[:12]
            self.jobs[job_id] = dict(id=job_id, clip_id=clip_id, label=clip['label'], kind='export', status='queued')
        self.pool.submit(self.export_worker, job_id, clip)
        return self.jobs[job_id]

    def job_update(self, job_id, **values):
        with self.lock:
            self.jobs[job_id].update(values)
            if self.jobs[job_id]['status'] in ('done', 'error'):
                atomic_json(OUTPUT/job_id/'job.json', self.jobs[job_id])

    def export_worker(self, job_id, clip):
        self.job_update(job_id, status='running')
        folder = OUTPUT/job_id
        folder.mkdir()
        source = self.sources[clip['source_id']]
        try:
            result = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-y',
                                     '-ss', str(clip['start_s']), '-i', str(REPO/source['path']), '-t', str(clip['duration']),
                                     '-map', '0:v:0', '-map', '0:a?', '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast',
                                     '-crf', '20', '-pix_fmt', 'yuv420p', '-c:a', 'aac', '-movflags', '+faststart',
                                     str(folder/'clip.mp4')], capture_output=True, text=True, timeout=180)
            if result.returncode:
                raise RuntimeError(result.stderr[-2000:])
            atomic_json(folder/'selection.json', dict(clip, original_source=source, export_file=str(folder/'clip.mp4')))
            self.job_update(job_id, status='done', url=f'/exports/{job_id}/clip.mp4', manifest_url=f'/exports/{job_id}/selection.json')
        except Exception as error:
            self.job_update(job_id, status='error', error=str(error))

    def simulate(self, clip_id):
        with self.lock:
            if self.simulation_busy:
                raise ValueError('Já há um trecho sendo simulado; aguarde')
            clip = next(c.copy() for c in self.clips if c['id'] == clip_id)
            if not clip.get('bag'):
                raise ValueError('Escolha uma gravação da ZED ou RealSense com bag para simular')
            with urlopen('http://127.0.0.1:8791/state', timeout=3) as response:
                state = json.load(response)
            with urlopen('http://127.0.0.1:8792/state', timeout=3) as response:
                replay = json.load(response)
            if state.get('simulation_only') is not True or state.get('domain') != 188:
                raise ValueError('Pare a simulação no painel 8791 antes de executar um corte')
            if replay.get('mode') != 'recorded' or not replay.get('bag', '').endswith('/'+clip['bag']):
                self.load_source(clip['source_id'])
            if state.get('phase')!='idle':self.robot_control(dict(action='stop'))
            self.simulation_busy = True
            job_id = uuid.uuid4().hex[:12]
            self.jobs[job_id] = dict(id=job_id, clip_id=clip_id, label=clip['label'], kind='simulate', status='queued')
        self.pool.submit(self.simulation_worker, job_id, clip)
        return self.jobs[job_id]

    def simulation_worker(self, job_id, clip):
        folder = OUTPUT/job_id
        folder.mkdir()
        self.job_update(job_id, status='running')
        try:
            root = '/home/spot-teleop/'
            script = root+str(HERE.relative_to(REPO))+'/'
            target = root+str(folder.relative_to(REPO))
            command_prefix = ['docker', 'exec', 'fr3-bag-control', 'bash', '-c',
                              'source /opt/ros/humble/setup.bash; exec python3 "$@"', 'editor']
            # Compute expected hand deltas from actual bag samples, never invent zeros.
            subprocess.run(command_prefix+[script+'prepare_clip_media.py', '--manifest', root+clip['bag'],
                            '--start', str(clip['bag_start_s']), '--end', str(clip['bag_end_s']),
                            '--label', clip['label'], '--output', target+'/clips.json'], check=True, capture_output=True, text=True, timeout=60)
            with (folder/'simulation.log').open('w') as log:
                subprocess.run(command_prefix+[script+'run_clips.py', '--clips', target+'/clips.json',
                                '--out-base', target, '--run-id', 'simulation'], stdout=log, stderr=subprocess.STDOUT,
                               check=True, timeout=clip['duration']+120)
            video = folder/'simulation/clip_01.mp4'
            subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-y', '-i', str(video),
                            '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-pix_fmt', 'yuv420p',
                            '-movflags', '+faststart', str(folder/'clip.mp4')], check=True, capture_output=True, timeout=120)
            summary = json.loads((folder/'simulation/summary.json').read_text())
            self.job_update(job_id, status='done', url=f'/exports/{job_id}/clip.mp4',
                            manifest_url=f'/exports/{job_id}/simulation/summary.json',
                            warning=summary['clips'][0].get('error'))
        except Exception as error:
            if isinstance(error, subprocess.CalledProcessError) and error.stderr:
                message = str(error.stderr)[-2000:]
            else:
                message = str(error)
            self.job_update(job_id, status='error', error=message)
        finally:
            with self.lock:
                self.simulation_busy = False


def handler(editor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send_json(self, code, data):
            payload = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def file(self, path, head=False):
            if not path.is_file():
                return self.send_error(404)
            size = path.stat().st_size
            try:
                start, end, partial = byte_range(self.headers.get('Range'), size)
            except ValueError:
                self.send_response(416)
                self.send_header('Content-Range', f'bytes */{size}')
                self.end_headers()
                return
            self.send_response(206 if partial else 200)
            self.send_header('Content-Type', mimetypes.guess_type(str(path))[0] or 'application/octet-stream')
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Content-Length', str(end-start+1))
            self.send_header('Cache-Control', 'no-cache')
            if partial:
                self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            self.end_headers()
            if head:
                return
            try:
                with path.open('rb') as stream:
                    stream.seek(start)
                    remaining = end-start+1
                    while remaining:
                        chunk = stream.read(min(262144, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_HEAD(self):
            self.do_GET(head=True)

        def do_GET(self, head=False):
            path = unquote(urlsplit(self.path).path)
            if path=='/review':return self.file(HERE/'movement_review.html',head)
            if path=='/api/review':return self.send_json(200,editor.review(parse_qs(urlsplit(self.path).query).get('dataset',['realsense'])[0]))
            if path in ('/', '/editor'):
                return self.file(HERE/'clip_editor.html', head)
            if path == '/api/library':
                return self.send_json(200, editor.library())
            if path == '/api/selections.json':
                return self.send_json(200, editor.clips)
            if path == '/api/demonstrations.json':
                return self.send_json(200, editor.demonstrations)
            if path == '/api/robot/state':
                try:return self.send_json(200,editor.robot_state())
                except (OSError,ValueError) as error:return self.send_json(503,dict(error=str(error)))
            if path == '/robot.jpg':
                try:
                    with urlopen('http://127.0.0.1:8791/view.jpg',timeout=2) as response:payload=response.read()
                    self.send_response(200);self.send_header('Content-Type','image/jpeg')
                    self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(payload)))
                    self.end_headers();self.wfile.write(payload);return
                except OSError:return self.send_error(503)
            if path == '/teaching.js':
                return self.file(HERE/'clip_teaching.js',head)
            for prefix, field in [('/media/', 'path'), ('/thumbnail/', 'thumbnail')]:
                if path.startswith(prefix):
                    source = editor.sources.get(path[len(prefix):])
                    if source:
                        return self.file(REPO/source[field], head)
                    return self.send_error(404)
            if path.startswith('/exports/'):
                target = (OUTPUT/path[len('/exports/'):]).resolve()
                if target.is_relative_to(OUTPUT.resolve()):
                    return self.file(target, head)
            self.send_error(404)

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (self.headers.get('X-Clip-Editor') != '1'
                    or (origin and urlsplit(origin).netloc != self.headers.get('Host'))):
                return self.send_json(403, dict(error='Origem recusada'))
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('Requisição inválida')
                data = json.loads(self.rfile.read(length))
                if self.path=='/api/review/feedback':return self.send_json(200,editor.review_feedback(data))
                if self.path == '/api/source/load':
                    return self.send_json(200,editor.load_source(data['source_id']))
                if self.path == '/api/clips':
                    return self.send_json(200, editor.save(data))
                if self.path == '/api/delete':
                    editor.remove(data['id'])
                    return self.send_json(200, dict(ok=True))
                if self.path == '/api/export':
                    return self.send_json(202, editor.export(data['id']))
                if self.path == '/api/simulate':
                    return self.send_json(202, editor.simulate(data['id']))
                if self.path == '/api/robot/control':
                    return self.send_json(200,editor.robot_control(data))
                if self.path == '/api/teaching/capture':
                    return self.send_json(200,editor.capture_pose(data))
                if self.path == '/api/teaching/remove':
                    return self.send_json(200,editor.remove_pose(data))
                self.send_error(404)
            except (ValueError, KeyError, TypeError, StopIteration, OSError) as error:
                self.send_json(400, dict(error=str(error) or 'Item não encontrado'))
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8793)
    args = parser.parse_args()
    editor = Editor()
    servers = []
    for host in ('127.0.0.1', '100.88.134.127'):
        server = ThreadingHTTPServer((host, args.port), handler(editor))
        servers.append(server)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Clip editor ready: http://100.88.134.127:{args.port}/', flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        for server in servers:
            server.shutdown()


if __name__ == '__main__':
    main()

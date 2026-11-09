#!/usr/bin/env python3
"""Play a recorded ZED protocol stage as a real-time camera on the real robot.

Reads <stage>/d8 (mcap) and publishes rebased, wall-clock-fresh images on
DDS domain 8 under /fr3_bagtest/{rgb,depth,camera_info}, plus the recorded
teleop samples under /fr3_bagtest/recorded_sample for comparison with a new
inference pass. A test operator_hand_tracker instance pointed at these topics
sees the bag exactly as if it were the live ZED3 camera.

Runs inside fr3-curobo (ROS 2 Humble): it needs rosbag2_py + cv2 and imports
the compressedDepth decoder from the lab remote_sim sources.

  python3 bag_camera.py /home/spot-teleop/bags/zed_protocol_20261030_005244/04_frente_tras [--play]

HTTP visualization on --port (default 8793): / shows the latest replay frame,
/state the player state, /control play/pause/restart.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from array import array
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REMOTE_SIM = Path(os.environ.get(
    'BAGTEST_REMOTE_SIM',
    '/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim'))
sys.path.insert(0, str(REMOTE_SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from bag_replay import decode_depth  # noqa: E402 (lab remote_sim)
except ImportError as error:
    sys.exit('bag_camera needs the lab remote_sim sources (bag_replay.decode_depth): %s' % error)
from bagtime import rebase_packet, validate_window, window_decision  # noqa: E402

import cv2
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
import rosbag2_py
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String

RGB = '/zed3/zed_node/rgb/image_rect_color/compressed'
DEPTH = '/zed3/zed_node/depth/depth_registered'
DEPTH_C = DEPTH + '/compressedDepth'
INFO = '/zed3/zed_node/depth/camera_info'
SAMPLE = '/fr3_operator/teleop_sample'
PREFIX = '/fr3_bagtest'


class Camera:
    def __init__(self, args):
        self.args = args
        self.open_reader()
        self.node = rclpy.create_node('fr3_bagtest_camera')
        self.pubs = {
            'rgb': self.node.create_publisher(Image, PREFIX + '/rgb', qos_profile_sensor_data),
            'depth': self.node.create_publisher(Image, PREFIX + '/depth', qos_profile_sensor_data),
            'camera_info': self.node.create_publisher(CameraInfo, PREFIX + '/camera_info', 10),
            'recorded_sample': self.node.create_publisher(String, PREFIX + '/recorded_sample',
                                                           qos_profile_sensor_data),
        }
        self.condition = threading.Condition()
        self.playing = False
        self.ended = False
        self.stop = False
        self.offset_ns = 0
        self.elapsed = 0.0
        self.start_s = 0.0
        self.end_s = self.duration
        self.counts = {}
        self.jpeg = None
        self.error = None
        self.pending_bag = None
        self.restart_requested = False
        self.worker = threading.Thread(target=self.run, daemon=True)

    def open_reader(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.args.bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        available = {t.topic_metadata.name for t in reader.get_metadata().topics_with_message_count}
        for need in (RGB, INFO):
            if need not in available:
                raise ValueError('Bag lacks ' + need)
        if DEPTH not in available and DEPTH_C not in available:
            raise ValueError('Bag lacks depth')
        topics = [RGB, INFO, SAMPLE, DEPTH, DEPTH_C]
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in available]))
        metadata = reader.get_metadata()
        self.reader = reader
        self.start_ns = metadata.starting_time.nanoseconds
        self.duration = metadata.duration.nanoseconds * 1e-9

    def map_stamp(self, stamp):
        return self.offset_ns + round((stamp - self.start_ns) / self.args.rate)

    def action(self, action, start_s=None, end_s=None, bag=None):
        if action == 'load':
            target = Path(bag or '')
            if not target.is_absolute():
                target = Path('/home/spot-teleop/bags') / target
            if (target / 'd8').is_dir():
                target = target / 'd8'
            root = Path('/home/spot-teleop/bags').resolve()
            try:
                resolved = target.resolve()
                ok = resolved.is_relative_to(root) and (resolved / 'metadata.yaml').is_file()
            except OSError:
                ok = False
            if not ok:
                raise ValueError('Expected an existing bag under /home/spot-teleop/bags')
        with self.condition:
            if action == 'load':
                self.pending_bag = resolved
                self.playing = False
                self.ended = False
                self.elapsed = 0.0
                self.start_s = 0.0
                self.counts = {}
                self.error = self.jpeg = None
                self.duration = 0.0
                self.end_s = 0.0
                self.restart_requested = True
            elif action == 'play':
                if self.ended:
                    raise ValueError('Bag ended: restart to repeat')
                if not self.playing:
                    self.offset_ns = time.time_ns() - round(self.elapsed * 1e9 / self.args.rate)
                    self.playing = True
            elif action == 'pause':
                if self.playing:
                    self.elapsed = (time.time_ns() - self.offset_ns) * 1e-9 * self.args.rate
                self.playing = False
            elif action == 'restart':
                if start_s is not None or end_s is not None:
                    self.start_s, self.end_s = validate_window(
                        start_s or 0.0, self.duration if end_s is None else end_s, self.duration)
                    self.elapsed = self.start_s
                else:
                    self.start_s, self.end_s, self.elapsed = 0.0, self.duration, 0.0
                self.playing = False
                self.ended = False
                self.restart_requested = True
            else:
                raise ValueError('Unknown action')
            self.condition.notify_all()

    def state(self):
        elapsed = (time.time_ns() - self.offset_ns) * 1e-9 * self.args.rate if self.playing else self.elapsed
        return dict(bag=str(self.args.bag), playing=self.playing, ended=self.ended,
                    elapsed_s=round(min(elapsed, self.end_s), 2), duration_s=round(self.duration, 2),
                    rate=self.args.rate, published=self.counts, error=self.error, domain=8,
                    topics=dict(rgb=PREFIX + '/rgb', depth=PREFIX + '/depth',
                                camera_info=PREFIX + '/camera_info',
                                recorded_sample=PREFIX + '/recorded_sample'))

    def header(self, message):
        ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        sec, nsec = divmod(self.map_stamp(ns), 10**9)
        message.header.stamp.sec, message.header.stamp.nanosec = sec, nsec
        return message.header

    def publish(self, name, message):
        self.pubs[name].publish(message)
        self.counts[name] = self.counts.get(name, 0) + 1

    def process(self, topic, data):
        if topic == SAMPLE:
            packet = rebase_packet(json.loads(deserialize_message(data, String).data),
                                   self.start_ns, self.offset_ns)
            self.publish('recorded_sample', String(data=json.dumps(packet, allow_nan=False)))
        elif topic == RGB:
            source = deserialize_message(data, CompressedImage)
            self.jpeg = bytes(source.data)
            pixels = cv2.imdecode(np.frombuffer(source.data, np.uint8), cv2.IMREAD_COLOR)
            if pixels is None:
                raise ValueError('Invalid JPEG')
            self.publish('rgb', Image(header=self.header(source), height=pixels.shape[0],
                                      width=pixels.shape[1], encoding='bgr8',
                                      step=pixels.shape[1] * 3, data=array('B', pixels.tobytes())))
        elif topic == INFO:
            message = deserialize_message(data, CameraInfo)
            self.header(message)
            self.publish('camera_info', message)
        elif topic == DEPTH:
            message = deserialize_message(data, Image)
            self.header(message)
            self.publish('depth', message)
        elif topic == DEPTH_C:
            source = deserialize_message(data, CompressedImage)
            pixels = decode_depth(source)
            self.publish('depth', Image(header=self.header(source), height=pixels.shape[0],
                                        width=pixels.shape[1], encoding='32FC1',
                                        step=pixels.shape[1] * 4, data=array('B', pixels.tobytes())))

    def run(self):
        try:
            while not self.stop:
                with self.condition:
                    if self.restart_requested:
                        if self.pending_bag is not None:
                            self.args.bag = self.pending_bag
                            self.pending_bag = None
                        self.open_reader()
                        self.end_s = self.duration
                        self.elapsed = self.start_s
                        self.restart_requested = False
                        self.ended = False
                        self.counts = {}
                        self.error = self.jpeg = None
                        if self.playing:
                            self.offset_ns = time.time_ns() - round(self.elapsed * 1e9 / self.args.rate)
                    if self.ended or not self.playing:
                        self.condition.wait(0.2)
                        continue
                    if not self.reader.has_next():
                        self.elapsed = self.end_s
                        self.playing, self.ended = False, True
                        print(json.dumps(self.state()), flush=True)
                        continue
                    topic, data, receipt = self.reader.read_next()
                    decision = window_decision(receipt, self.start_ns, self.start_s, self.end_s)
                    if decision == 'skip':
                        continue
                    if decision == 'end':
                        self.elapsed = self.end_s
                        self.playing, self.ended = False, True
                        print(json.dumps(self.state()), flush=True)
                        continue
                    while not self.stop:
                        if self.restart_requested or not self.playing:
                            break
                        delay = (self.map_stamp(receipt) - time.time_ns()) * 1e-9
                        if delay <= 0:
                            break
                        self.condition.wait(min(delay, 0.2))
                    if self.stop or self.restart_requested or not self.playing:
                        continue
                    self.process(topic, data)
        except Exception as error:
            self.error = str(error)
            print('CAMERA ERROR:', self.error, flush=True)
        finally:
            self.playing, self.ended = False, True
            print(json.dumps(self.state()), flush=True)


PAGE = '''<!doctype html><meta charset="utf-8"><title>Bag como camera · FR3</title>
<style>body{background:#14161b;color:#eee;font:18px sans-serif;max-width:960px;margin:30px auto}button{margin:10px;padding:12px}img{width:640px;max-width:100%}pre{white-space:pre-wrap}</style>
<h1>Bag como camera do robo real (domínio 8)</h1>
<p>O rastreador de teste le estes topicos como se fossem a ZED3 ao vivo. Taxa 1.0 = tempo real.</p>
<button onclick="act('play')">Tocar</button><button onclick="act('pause')">Pausar</button><button onclick="act('restart')">Voltar ao inicio</button>
<img id="rgb"><pre id="state"></pre>
<script>async function act(action){const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json','X-Bag-Camera':'1'},body:JSON.stringify({action})});if(!r.ok)alert(await r.text())}
setInterval(async()=>{try{state.textContent=JSON.stringify(await(await fetch('/state')).json(),null,2);rgb.src='/rgb.jpg?t='+Date.now()}catch(e){}},500)</script>'''


def handler(camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, mime):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                return self.send(200, PAGE.encode(), 'text/html; charset=utf-8')
            if path == '/state':
                return self.send(200, json.dumps(camera.state()).encode(), 'application/json')
            if path == '/rgb.jpg' and camera.jpeg:
                return self.send(200, camera.jpeg, 'image/jpeg')
            self.send(404, b'Waiting for replay', 'text/plain')

        def do_POST(self):
            if self.path != '/control' or self.headers.get('X-Bag-Camera') != '1':
                return self.send(403, b'Forbidden', 'text/plain')
            try:
                payload = json.loads(self.rfile.read(min(int(self.headers.get('Content-Length', '0')), 1024)))
                camera.action(payload['action'], bag=payload.get('bag'))
                self.send(200, b'{"ok":true}', 'application/json')
            except (ValueError, KeyError, TypeError) as error:
                self.send(409, str(error).encode(), 'text/plain')
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--port', type=int, default=8793)
    parser.add_argument('--play', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not 0.1 <= args.rate <= 2:
        parser.error('rate must be in [0.1, 2]')
    if (args.bag / 'd8').is_dir():
        args.bag = args.bag / 'd8'
    rclpy.init(domain_id=8)
    camera = Camera(args)
    servers = []
    for host in ('127.0.0.1', '100.88.134.127'):
        server = ThreadingHTTPServer((host, args.port), handler(camera))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
    camera.worker.start()
    if args.play:
        camera.action('play')
    print('Bag camera, DDS 8, paused=%s, HTTP %d' % (not args.play, args.port), flush=True)
    try:
        rclpy.spin(camera.node)
    except KeyboardInterrupt:
        pass
    finally:
        with camera.condition:
            camera.stop = True
            camera.condition.notify_all()
        camera.worker.join(timeout=5)
        for server in servers:
            server.shutdown()
        camera.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

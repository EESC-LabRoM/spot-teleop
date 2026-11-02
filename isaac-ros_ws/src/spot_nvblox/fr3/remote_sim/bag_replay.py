"""Replay only operator data into DDS 188. No UDP or physical command topics.

The bag receipt timeline is rebased to wall time; acquisition delays and paired
timestamps are preserved. Never publish the recorded robot joints as sim state.
"""
import argparse
from array import array
import copy
import json
import math
from pathlib import Path
import struct
import threading
import time
from urllib.request import urlopen
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
import rosbag2_py
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from teleop_mapping import paired_sample

RGB = '/zed3/zed_node/rgb/image_rect_color/compressed'
DEPTH = '/zed3/zed_node/depth/depth_registered'
INFO = '/zed3/zed_node/depth/camera_info'
SAMPLE = '/fr3_operator/teleop_sample'
TRACKING = '/fr3_operator/tracking'
PREFIX = '/fr3_sim/operator'
RS_RGB='/realsense/rgb/image_rect/compressed'
RS_DEPTH='/realsense/depth/image_rect'
RS_INFO='/realsense/camera_info'

def source_topics(available):
    if RS_RGB in available:return RS_RGB,RS_DEPTH,RS_INFO
    if RGB in available:return RGB,DEPTH,INFO
    raise ValueError('Bag must contain ZED or RealSense RGB')


def rebase_packet(packet, map_stamp):
    result = copy.deepcopy(packet)
    for key in ('stamp_ns', 'orientation_stamp_ns', 'roll_stamp_ns'):
        if result.get(key) is not None:
            result[key] = map_stamp(int(result[key]))
    return result


def validate_seek_window(start_s, end_s, duration):
    if isinstance(start_s, bool) or isinstance(end_s, bool):
        raise ValueError('start_s and end_s must be finite numbers')
    try:
        start_val = float(start_s)
        end_val = float(end_s)
    except (TypeError, ValueError) as error:
        raise ValueError('start_s and end_s must be finite numbers') from error
    if not (math.isfinite(start_val) and math.isfinite(end_val)):
        raise ValueError('start_s and end_s must be finite numbers')
    # Editor cuts are stored to milliseconds; clamp only rounding at the end.
    duration=float(duration)
    if end_val<=duration+.0005+1e-9:end_val=min(end_val,duration)
    if not (0.0 <= start_val < end_val <= duration):
        raise ValueError('Require 0 <= start_s < end_s <= duration')
    return start_val, end_val


def window_decision(stamp_ns, start_ns, start_s=0.0, end_s=None):
    start_limit_ns = int(start_ns) + round(float(start_s) * 1e9)
    if int(stamp_ns) < start_limit_ns:
        return 'skip'
    if end_s is not None:
        end_limit_ns = int(start_ns) + round(float(end_s) * 1e9)
        if int(stamp_ns) > end_limit_ns:
            return 'end'
    return 'deliver'


def filter_window_messages(messages, start_ns, start_s, end_s):
    """Pure window helper: skips messages before start_s, delivers in [start_s, end_s], ends after end_s."""
    delivered = []
    skipped = 0
    ended = False
    for item in messages:
        _, _, stamp_ns = item
        decision = window_decision(stamp_ns, start_ns, start_s, end_s)
        if decision == 'skip':
            skipped += 1
            continue
        if decision == 'end':
            ended = True
            break
        delivered.append(item)
    else:
        ended = True
    return delivered, skipped, ended


def decode_depth(message):
    """ROS compressedDepth: 12-byte config then lossless PNG.

    32FC1 uses inverse-depth quantization; 16UC1 stores millimetres directly.
    """
    data = bytes(message.data)
    if data[12:20] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Expected compressedDepth PNG with 12-byte header')
    raw = cv2.imdecode(np.frombuffer(data[12:], np.uint8), cv2.IMREAD_UNCHANGED)
    if raw is None or raw.dtype != np.uint16:
        raise ValueError('Invalid compressedDepth PNG')
    if message.format.startswith('32FC1'):
        _, a, b = struct.unpack('<iff', data[:12])
        valid = (raw > 0) & (raw.astype(np.float32) > b)
        depth = np.full(raw.shape, np.nan, np.float32)
        depth[valid] = a / (raw[valid].astype(np.float32) - b)
        return depth
    if message.format.startswith('16UC1'):
        depth = raw.astype(np.float32) * .001
        depth[raw == 0] = np.nan
        return depth
    raise ValueError('Unsupported compressedDepth format: ' + message.format)


class Replay:
    def __init__(self, args):
        self.args = args
        self.open_reader()
        self.node = rclpy.create_node('fr3_bag_operator_replay')
        self.publishers = {name: self.node.create_publisher(kind, PREFIX+'/'+name,
                                                           10 if name == 'camera_info' else qos_profile_sensor_data)
                           for name, kind in [('rgb', Image), ('depth', Image), ('camera_info', CameraInfo),
                                              ('sample', String), ('tracking', String), ('overlay', Image),
                                              ('recorded_sample', String)]}
        self.condition = threading.Condition()
        self.playing = False
        self.ended = False
        self.stop = False
        self.offset_ns = 0
        self.elapsed = 0.
        self.start_s = 0.
        self.end_s = self.duration
        self.window_end_s = None
        self.counts = {}
        self.paired = 0
        self.position_only = 0
        self.error = None
        self.restart_requested = False
        self.seek_requested = False
        self.jpeg = None
        self.worker = threading.Thread(target=self.run, daemon=True)

    def open_reader(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.args.bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        metadata=reader.get_metadata()
        available={t.topic_metadata.name for t in metadata.topics_with_message_count}
        rgb,depth,info=source_topics(available)
        if SAMPLE not in available:raise ValueError('Bag must contain teleop_sample')
        if self.args.mode=='retrack' and (info not in available or not ({depth,depth+'/compressedDepth'} & available)):
            raise ValueError('Retracking requires original RGB-D and camera_info; this bag is replay-only')
        reader.set_filter(rosbag2_py.StorageFilter(topics=[rgb,SAMPLE,TRACKING,info,depth,depth+'/compressedDepth']))
        self.reader,self.metadata=reader,metadata
        self.rgb_topic,self.depth_topic,self.info_topic=rgb,depth,info
        self.start_ns=metadata.starting_time.nanoseconds
        self.duration=metadata.duration.nanoseconds*1e-9

    def map_stamp(self, stamp):
        return self.offset_ns + round((stamp-self.start_ns) / self.args.rate)

    def action(self, action, start_s=None, end_s=None, bag=None):
        if action in ('restart','load'):
            if self.args.mode != 'recorded':
                raise ValueError('Retrack: recrie o container para reiniciar também os filtros e a calibração')
            try:
                with urlopen('http://127.0.0.1:8791/state', timeout=2) as response:
                    state = json.load(response)
                if state.get('simulation_only') is not True or state.get('domain') != 188 or state.get('phase') != 'idle':
                    raise ValueError('Clique Parar no painel do robô antes de voltar ao início')
            except OSError as error:
                raise ValueError('Não foi possível confirmar que a simulação está parada') from error
        if action == 'seek':
            start_val, end_val = validate_seek_window(start_s, end_s, self.duration)
        if action=='load':
            target=Path(bag or '').resolve();root=(Path(__file__).resolve().parents[5]/'bags').resolve()
            if not target.is_relative_to(root) or not (target/'metadata.yaml').is_file():
                raise ValueError('Expected an existing bag under spot-teleop/bags')
        with self.condition:
            if action=='load':
                original=self.args.bag
                try:
                    self.args.bag=target;self.open_reader()
                except Exception:
                    self.args.bag=original;raise
                self.playing=False;self.ended=False;self.elapsed=0.;self.start_s=0.;self.end_s=self.duration;self.window_end_s=None
                self.counts={};self.paired=self.position_only=0;self.jpeg=None;self.error=None
                self.restart_requested=False;self.seek_requested=True
            elif action == 'play':
                if self.ended:
                    raise ValueError('Fim da bag: reinicie o replay para repetir')
                if not self.playing:
                    self.offset_ns = time.time_ns()-round(self.elapsed*1e9/self.args.rate)
                    self.playing = True
            elif action == 'pause':
                if self.playing:
                    self.elapsed = (time.time_ns()-self.offset_ns)*1e-9*self.args.rate
                self.playing = False
            elif action == 'restart':
                self.playing = False
                self.start_s = 0.
                self.end_s = self.duration
                self.window_end_s = None
                self.elapsed = 0.
                self.ended = False
                self.restart_requested = True
                self.seek_requested = False
            elif action == 'seek':
                self.playing = False
                self.start_s = start_val
                self.end_s = end_val
                self.window_end_s = end_val
                self.elapsed = start_val
                self.ended = False
                self.seek_requested = True
                self.restart_requested = False
            else:
                raise ValueError('Unknown action')
            self.condition.notify_all()

    def state(self):
        elapsed = (time.time_ns()-self.offset_ns)*1e-9*self.args.rate if self.playing else self.elapsed
        return dict(bag=str(self.args.bag), mode=self.args.mode, playing=self.playing, ended=self.ended,
                    elapsed_s=round(min(elapsed, self.end_s), 2), duration_s=self.duration,
                    start_s=self.start_s, end_s=self.end_s,
                    rate=self.args.rate, published=self.counts, paired_samples=self.paired,
                    position_only_samples=self.position_only, error=self.error, domain=188)

    def header(self, message):
        ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(self.map_stamp(ns), 10**9)
        return message.header

    def publish(self, name, message):
        self.publishers[name].publish(message)
        self.counts[name] = self.counts.get(name, 0)+1

    def process(self, topic, data):
        if topic in (SAMPLE, TRACKING):
            packet = rebase_packet(json.loads(deserialize_message(data, String).data), self.map_stamp)
            message = String(data=json.dumps(packet, allow_nan=False))
            if topic == SAMPLE:
                if paired_sample(packet) is None:
                    self.position_only += 1
                else:
                    self.paired += 1
                self.publish('sample' if self.args.mode == 'recorded' else 'recorded_sample', message)
            elif self.args.mode == 'recorded':
                self.publish('tracking', message)
        elif topic == self.rgb_topic:
            source = deserialize_message(data, CompressedImage)
            self.jpeg = bytes(source.data)
            pixels = cv2.imdecode(np.frombuffer(source.data, np.uint8), cv2.IMREAD_COLOR)
            if pixels is None:
                raise ValueError('Invalid JPEG')
            out = Image(header=self.header(source), height=pixels.shape[0], width=pixels.shape[1],
                        encoding='bgr8', step=pixels.shape[1]*3, data=array('B', pixels.tobytes()))
            self.publish('rgb', out)
            if self.args.mode == 'recorded':
                self.publish('overlay', out)
        elif topic == self.info_topic:
            message = deserialize_message(data, CameraInfo)
            self.header(message)
            self.publish('camera_info', message)
        elif self.args.mode == 'retrack' and topic == self.depth_topic:
            message = deserialize_message(data, Image)
            self.header(message)
            self.publish('depth', message)
        elif self.args.mode == 'retrack' and topic == self.depth_topic+'/compressedDepth':
            source = deserialize_message(data, CompressedImage)
            pixels = decode_depth(source)
            message = Image(header=self.header(source), height=pixels.shape[0], width=pixels.shape[1],
                            encoding='32FC1', step=pixels.shape[1]*4, data=array('B', pixels.tobytes()))
            self.publish('depth', message)

    def run(self):
        try:
            while not self.stop:
                with self.condition:
                    if self.restart_requested:
                        self.open_reader()
                        self.restart_requested = False
                        self.ended = False
                        self.elapsed = 0.
                        self.counts = {}
                        self.paired = self.position_only = 0
                        self.error = self.jpeg = None
                        if self.playing:
                            self.offset_ns = time.time_ns()-round(self.elapsed*1e9/self.args.rate)
                    if self.seek_requested:
                        self.open_reader()
                        target_ns = self.start_ns + round(self.start_s * 1e9)
                        if hasattr(self.reader, 'seek'):
                            self.reader.seek(target_ns)
                        self.seek_requested = False
                        self.ended = False
                        self.elapsed = self.start_s
                        self.counts = {}
                        self.paired = self.position_only = 0
                        self.error = None
                        if self.playing:
                            self.offset_ns = time.time_ns()-round(self.elapsed*1e9/self.args.rate)
                    if self.ended or not self.playing:
                        self.condition.wait(.2)
                        continue
                    if not self.reader.has_next():
                        self.elapsed = self.end_s
                        self.playing = False
                        self.ended = True
                        print(json.dumps(self.state()), flush=True)
                        continue
                    topic, data, receipt = self.reader.read_next()
                    decision = window_decision(receipt, self.start_ns, self.start_s, self.window_end_s)
                    if decision == 'skip':
                        continue
                    if decision == 'end':
                        self.elapsed = self.end_s
                        self.playing = False
                        self.ended = True
                        print(json.dumps(self.state()), flush=True)
                        continue
                    while not self.stop:
                        if self.restart_requested or self.seek_requested:
                            break
                        if not self.playing:
                            self.condition.wait(.2)
                            continue
                        delay = (self.map_stamp(receipt)-time.time_ns())*1e-9
                        if delay <= 0:
                            break
                        self.condition.wait(min(delay, .2))
                    if self.stop:
                        return
                    if self.restart_requested or self.seek_requested:
                        continue
                    self.process(topic, data)
        except Exception as error:
            self.error = str(error)
            print('REPLAY ERROR:', self.error, flush=True)
        finally:
            self.playing = False
            self.ended = True
            print(json.dumps(self.state()), flush=True)


PAGE = '''<!doctype html><meta charset="utf-8"><title>Replay da mão → Franka</title>
<style>body{background:#191b20;color:#eee;font:18px sans-serif;max-width:960px;margin:30px auto}button,a{margin:10px;padding:12px}img{width:640px;max-width:100%}pre{white-space:pre-wrap}</style>
<h1>Replay da mão → Franka simulado</h1>
<p>1. Abra o <a id="sim">painel do robô</a> e espere o controle ficar pronto.<br>
2. Clique Start teleop da bag no painel do robô.<br>3. Toque a bag aqui. O movimento começa após uma referência estável.</p>
<button onclick="act('play')">Tocar / continuar</button><button onclick="act('pause')">Pausar bag</button><button onclick="act('restart')">Voltar ao início (recorded)</button>
<p>Para repetir: Parar no painel do robô → voltar ao início → ajustar ganhos → Start → tocar. No modo retrack, recrie o container para zerar os filtros.</p>
<img id="rgb"><pre id="state"></pre>
<script>sim.href=location.protocol+'//'+location.hostname+':8791/';
async function act(action){const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json','X-Bag-Control':'1'},body:JSON.stringify({action})});if(!r.ok)alert(await r.text())}
setInterval(async()=>{try{state.textContent=JSON.stringify(await(await fetch('/state')).json(),null,2);rgb.src='/rgb.jpg?t='+Date.now()}catch(e){}},500)</script>'''


def handler(replay):
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
                return self.send(200, json.dumps(replay.state()).encode(), 'application/json')
            if path == '/rgb.jpg' and replay.jpeg:
                return self.send(200, replay.jpeg, 'image/jpeg')
            self.send(404, b'Waiting for replay', 'text/plain')

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (self.path != '/control' or self.headers.get('X-Bag-Control') != '1'
                    or (origin and origin.split('://')[-1] != self.headers.get('Host'))):
                return self.send(403, b'Forbidden', 'text/plain')
            try:
                body = self.rfile.read(min(int(self.headers.get('Content-Length', '0')), 1024))
                payload = json.loads(body)
                if not isinstance(payload, dict):
                    raise ValueError('Invalid control payload')
                replay.action(payload['action'], payload.get('start_s'), payload.get('end_s'),payload.get('bag'))
                self.send(200, b'{"ok":true}', 'application/json')
            except (ValueError, KeyError, TypeError) as error:
                self.send(409, str(error).encode(), 'text/plain')
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--mode', choices=['recorded', 'retrack'], default='recorded')
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--port', type=int, default=8792)
    parser.add_argument('--play', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .1 <= args.rate <= 2:
        parser.error('rate must be in [0.1, 2]')
    if (args.bag/'d8').is_dir():
        args.bag = args.bag/'d8'
    rclpy.init(domain_id=188)
    replay = Replay(args)
    servers = []
    for host in ('127.0.0.1', '100.88.134.127'):
        server = ThreadingHTTPServer((host, args.port), handler(replay))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
    replay.worker.start()
    if args.play:
        replay.action('play')
    print(f'Replay {args.mode}, DDS 188, paused={not args.play}, HTTP {args.port}', flush=True)
    try:
        rclpy.spin(replay.node)
    except KeyboardInterrupt:
        pass
    finally:
        with replay.condition:
            replay.stop = True
            replay.condition.notify_all()
        replay.worker.join(timeout=5)
        for server in servers:
            server.shutdown()
        replay.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

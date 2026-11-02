"""Prepare seekable camera videos from ROS bags without starting ROS publishers."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
MEDIA = REPO / 'clips_out' / 'editor_media'


def extract(bag, output):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    output.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    meta = reader.get_metadata()
    start = meta.starting_time.nanoseconds
    duration = meta.duration.nanoseconds * 1e-9
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/zed3/zed_node/rgb/image_rect_color/compressed']))
    frames = []
    while reader.has_next():
        _, data, receipt = reader.read_next()
        message = deserialize_message(data, CompressedImage)
        name = f'{len(frames):06d}.jpg'
        (output / name).write_bytes(bytes(message.data))
        frames.append((name, (receipt-start)*1e-9))
    if not frames:
        raise ValueError('Bag has no ZED3 RGB frames')
    lines = ['ffconcat version 1.0']
    # The video clock is the bag receipt clock, including the initial RGB gap.
    if frames[0][1] > 0:
        lines += [f"file '{frames[0][0]}'", f'duration {frames[0][1]:.9f}']
    for i, (name, stamp) in enumerate(frames):
        end = frames[i+1][1] if i+1 < len(frames) else duration
        lines += [f"file '{name}'", f'duration {max(.000001, end-stamp):.9f}']
    lines += [f"file '{frames[-1][0]}'"]
    (output / 'frames.txt').write_text('\n'.join(lines)+'\n')
    (output / 'metadata.json').write_text(json.dumps(dict(duration=duration, frames=len(frames))))


def probe(path):
    result = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'json', str(path)], capture_output=True, text=True, check=True)
    return float(json.loads(result.stdout)['format']['duration'])


def video_fps(path):
    result = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                             'stream=avg_frame_rate', '-of', 'json', str(path)], capture_output=True, text=True, check=True)
    ratio = json.loads(result.stdout)['streams'][0]['avg_frame_rate']
    numerator, denominator = map(float, ratio.split('/'))
    return numerator/denominator if denominator else 25.


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--extract', type=Path)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--manifest', type=Path)
    ap.add_argument('--start', type=float)
    ap.add_argument('--end', type=float)
    ap.add_argument('--label', default='Trecho manual')
    args = ap.parse_args()
    if args.manifest:
        from make_clips import load_bag_samples, palm_roll_deg
        samples, duration = load_bag_samples(args.manifest)
        if not 0 <= args.start < args.end <= duration+.001:
            raise ValueError('Invalid bag selection')
        chosen = [s for s in samples if args.start <= s['t'] <= args.end]
        if not chosen:
            raise ValueError('O trecho não contém amostras válidas da mão')
        first, last = chosen[0], chosen[-1]
        delta = (last['p']-first['p'])*100
        clip = dict(id=1, label=args.label, start_s=args.start, end_s=args.end, duration=args.end-args.start,
                    motion_end_s=args.end, dx_cm=float(delta[0]), dy_cm=float(delta[1]), dz_cm=float(delta[2]),
                    roll_deg=palm_roll_deg(first['q'], last['q']))
        args.output.write_text(json.dumps([clip], ensure_ascii=False, indent=2))
        return
    if args.extract:
        extract(args.extract, args.output)
        return
    MEDIA.mkdir(parents=True, exist_ok=True)
    library = []
    for bag in sorted((REPO/'bags').glob('*/d8/metadata.yaml')):
        name = bag.parent.parent.name
        video = MEDIA / f'{name}.mp4'
        meta_path = MEDIA / f'{name}.json'
        if not video.exists() or not meta_path.exists():
            with tempfile.TemporaryDirectory(prefix='frames_', dir=MEDIA) as temporary:
                folder = Path(temporary)
                container_script = '/home/spot-teleop/'+str(Path(__file__).resolve().relative_to(REPO))
                container_bag = '/home/spot-teleop/'+str(bag.parent.relative_to(REPO))
                container_out = '/home/spot-teleop/'+str(folder.relative_to(REPO))
                subprocess.run(['docker', 'exec', 'fr3-bag-control', 'bash', '-c',
                                'source /opt/ros/humble/setup.bash; exec python3 "$@"', 'extract',
                                container_script, '--extract', container_bag, '--output', container_out], check=True)
                meta = json.loads((folder/'metadata.json').read_text())
                subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'concat', '-safe', '0',
                                '-i', str(folder/'frames.txt'), '-t', str(meta['duration']), '-vf', 'fps=25',
                                '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-crf', '21',
                                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)], check=True)
                meta_path.write_text(json.dumps(meta))
        meta = json.loads(meta_path.read_text())
        library.append(dict(id=name, label=name.split('_', 2)[-1]+' · câmera completa', group='Gravações da ZED',
                            path=str(video.relative_to(REPO)), duration=meta['duration'], bag=str(bag.parent.relative_to(REPO)),
                            bag_offset_s=0., frames=meta['frames']))
        print('Camera ready:', name, flush=True)
    for summary_path in sorted((REPO/'clips_out').glob('*/summary.json')):
        summary = json.loads(summary_path.read_text())
        for clip in summary.get('clips', []):
            video = summary_path.parent / (clip['clip_name']+'.mp4')
            if not video.exists():
                continue
            # Rendered comparisons include reset/calibration/tail. Their time is
            # not an exact affine mapping to the bag: do not invent bag offsets.
            library.append(dict(id=summary_path.parent.name+'_'+clip['clip_name'],
                                label=clip['clip_name']+' · '+clip['label'], group='Mão + robô · vídeos já simulados',
                                path=str(video.relative_to(REPO)), duration=probe(video), bag=None,
                                suggested_start_s=clip.get('start_s'), suggested_end_s=clip.get('end_s')))
    for source in library:
        video = REPO/source['path']
        source['fps'] = video_fps(video)
        sheet = MEDIA / (source['id']+'_timeline.jpg')
        if not sheet.exists():
            subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(video),
                            '-vf', f"fps={10/source['duration']},scale=144:81,tile=10x1", '-frames:v', '1',
                            '-threads', '2', str(sheet)], check=True)
        source['thumbnail'] = str(sheet.relative_to(REPO))
    target = MEDIA/'library.json'
    target.write_text(json.dumps(library, ensure_ascii=False, indent=2))
    print('Saved', len(library), 'sources:', target, flush=True)


if __name__ == '__main__':
    main()

"""Frame/CSV output shared by simulation backends."""
import json,math,os,time
from PIL import Image

class FrameRecorder:
    """Pure wall-time frame and CSV recorder for simulation views."""

    def __init__(self, quality=85):
        self.quality = int(quality)
        self.active = False
        self.out_dir = None
        self.fps = 15.0
        self.period = 1.0 / 15.0
        self.frame_idx = 0
        self.next_shot = None
        self._csv_file = None

    def start(self, out_dir, fps=15.0, now_mono=None):
        self.stop()
        if not out_dir or not isinstance(out_dir, str):
            raise ValueError("record start requires a directory path")
        fps_val = float(fps)
        if not math.isfinite(fps_val) or fps_val <= 0.0:
            raise ValueError("record fps must be > 0")
        self.out_dir = os.path.abspath(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        csv_path = os.path.join(self.out_dir, "frames.csv")
        self._csv_file = open(csv_path, "w", encoding="utf-8")
        self._csv_file.write("frame,sim_time,wall_time\n")
        self._csv_file.flush()
        self.fps = fps_val
        self.period = 1.0 / fps_val
        self.frame_idx = 0
        self.next_shot = None if now_mono is None else float(now_mono)
        self.active = True

    def stop(self):
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
        self.active = False
        self.next_shot = None

    def handle_command(self, payload, now_mono=None):
        data = json.loads(payload) if isinstance(payload, str) else payload
        if not isinstance(data, dict):
            raise ValueError("Invalid record command payload")
        cmd = data.get("cmd")
        if cmd == "start":
            self.start(data.get("dir"), data.get("fps", 15.0), now_mono=now_mono)
        elif cmd == "stop":
            self.stop()
        else:
            raise ValueError(f"Unknown record cmd: {cmd}")

    def should_capture(self, now_mono):
        if not self.active:
            return False
        if self.next_shot is None:
            return True
        return float(now_mono) >= self.next_shot

    def record_frame(self, rgb_image, sim_time, wall_time=None, now_mono=None):
        if not self.active or self._csv_file is None:
            return None
        w_time = float(time.time() if wall_time is None else wall_time)
        s_time = float(sim_time)
        idx = self.frame_idx
        jpg_path = os.path.join(self.out_dir, f"frame_{idx:05d}.jpg")
        Image.fromarray(rgb_image).save(jpg_path, format="JPEG", quality=self.quality)
        self._csv_file.write(f"{idx},{s_time:.6f},{w_time:.6f}\n")
        self._csv_file.flush()
        self.frame_idx += 1
        if now_mono is not None:
            mono = float(now_mono)
            if self.next_shot is None:
                self.next_shot = mono + self.period
            else:
                self.next_shot += self.period
                if mono >= self.next_shot:
                    self.next_shot = mono + self.period
        return jpg_path


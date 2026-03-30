import base64
import re
import site
import sys
import os
import io
import time
import threading
import json
import uuid
from collections import OrderedDict
from pathlib import Path

# Inject venv site-packages so torch/ultralytics are found when running via ros2 run.
# Use addsitedir (not sys.path.insert) so editable-install .pth files are processed.
_VENV_SITE = "/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/venv_valve_detection/lib/python3.10/site-packages"
if os.path.isdir(_VENV_SITE):
    site.addsitedir(_VENV_SITE)

import requests
from PIL import Image, ImageDraw, ImageFont

try:
    import numpy as np
except ImportError:
    np = None

# ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image as RosImage, CameraInfo
    from geometry_msgs.msg import TransformStamped, PointStamped
    from cv_bridge import CvBridge
    import cv2
    import tf2_ros
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    import message_filters
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Aviso: Bibliotecas ROS 2 (rclpy, sensor_msgs, cv_bridge) não encontradas. Modo câmera indisponível.")


# Imports de segmentacao - Ultralytics SAM2
try:
    import torch
    from ultralytics import SAM
    from ultralytics.models.sam import SAM2VideoPredictor
    SAM2_AVAILABLE = True
except ImportError:
    SAM2_AVAILABLE = False
    print("Aviso: Ultralytics SAM2 nao encontrado. Segmentacao SAM desativada.")
    SAM2VideoPredictor = None  # type: ignore

VLLM_URL = "http://100.111.174.61:8000"
# ===============================================
DEBUG_LOG_PATH = "/home/spot-teleop/spot-ros2_ws/.cursor/debug-13b6ac.log"
DEBUG_SESSION_ID = "13b6ac"


def _append_debug_log(hypothesis_id: str, location: str, message: str, data: dict):
    payload = {
        "sessionId": DEBUG_SESSION_ID,
        "id": f"log_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
        "timestamp": int(time.time() * 1000),
        "runId": "fov-trigger-odom-desync",
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
    }
    try:
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=True) + "\n")
    except Exception:
        pass


# Ultralytics SAM2 tiny (auto-download)
SAM2_MODEL_NAME = "sam2.1_t.pt"

# Global SAM Model (prompt segmentation)
sam_model = None
# Global SAM Video Tracker (stateful)
sam_video_model = None


if SAM2_AVAILABLE:

    class SAM2ROSVideoPredictor(SAM2VideoPredictor):
        """
        SAM2VideoPredictor com fluxo ROS (numpy frame a frame).
    
        O SAM2VideoPredictor da Ultralytics assume dataset em modo video (init_state)
        e usa dataset.frame; LoadPilAndNumpy so tem mode 'image'. Esta subclasse
        inicializa inference_state sem exigir video de arquivo e usa _ros_frame_idx
        como indice de frame continuo entre chamadas.
        """
    
        def __init__(self, cfg=None, overrides=None, _callbacks=None):
            from ultralytics.utils import DEFAULT_CFG
            if cfg is None:
                cfg = DEFAULT_CFG
            super().__init__(cfg, overrides, _callbacks)
            self.callbacks["on_predict_start"] = [
                self._init_state_ros if cb is SAM2VideoPredictor.init_state else cb
                for cb in self.callbacks["on_predict_start"]
            ]
            self._ros_frame_idx = 0
    
        @staticmethod
        def _init_state_ros(predictor):
            if len(predictor.inference_state) > 0:
                return
            assert predictor.dataset is not None
            ds = predictor.dataset
            num_frames = getattr(ds, "frames", 10**9)
            inference_state = {
                "num_frames": num_frames,
                "point_inputs_per_obj": {},
                "mask_inputs_per_obj": {},
                "constants": {},
                "obj_id_to_idx": OrderedDict(),
                "obj_idx_to_id": OrderedDict(),
                "obj_ids": [],
                "output_dict": {
                    "cond_frame_outputs": {},
                    "non_cond_frame_outputs": {},
                },
                "output_dict_per_obj": {},
                "temp_output_dict_per_obj": {},
                "consolidated_frame_inds": {
                    "cond_frame_outputs": set(),
                    "non_cond_frame_outputs": set(),
                },
                "tracking_has_started": False,
                "frames_already_tracked": [],
            }
            predictor.inference_state = inference_state
    
        def inference(self, im, bboxes=None, points=None, labels=None, masks=None):
            if self.dataset is not None:
                self.dataset.frame = self._ros_frame_idx
            try:
                return super().inference(im, bboxes=bboxes, points=points, labels=labels, masks=masks)
            finally:
                self._ros_frame_idx += 1
    
    
def load_sam_model():
    """Load Ultralytics SAM2 prompt model."""
    global sam_model
    if not SAM2_AVAILABLE:
        return None
    
    if sam_model is None:
        try:
            print(f"Carregando Ultralytics SAM2 ({SAM2_MODEL_NAME})...")
            sam_model = SAM(SAM2_MODEL_NAME)
        except Exception as e:
            print(f"Erro ao carregar SAM2: {e}")
            import traceback
            traceback.print_exc()
            return None
    return sam_model


def load_sam_video_model():
    """Load Ultralytics stateful SAM2 tracker."""
    global sam_video_model
    if not SAM2_AVAILABLE:
        return None
    
    if sam_video_model is None:
        try:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            print(f"Carregando Ultralytics SAM2 tracker ({SAM2_MODEL_NAME})...")
            sam_video_model = SAM2ROSVideoPredictor(
                overrides={
                    "conf": 0.01,
                    "task": "segment",
                    "mode": "predict",
                    "imgsz": 1024,
                    "model": SAM2_MODEL_NAME,
                    "save": False,
                    "verbose": False,
                },
            )
        except Exception as e:
            print(f"Erro ao carregar tracker SAM2: {e}")
            import traceback
            traceback.print_exc()
            return None
    return sam_video_model


def create_video_predictor():
    """Nova instância SAM2ROSVideoPredictor (um estado de vídeo por câmera). Não usar o singleton da mão."""
    if not SAM2_AVAILABLE:
        return None
    try:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        return SAM2ROSVideoPredictor(
            overrides={
                "conf": 0.01,
                "task": "segment",
                "mode": "predict",
                "imgsz": 1024,
                "model": SAM2_MODEL_NAME,
                "save": False,
                "verbose": False,
            },
        )
    except Exception as e:
        print(f"Erro ao criar SAM2ROSVideoPredictor: {e}")
        import traceback
        traceback.print_exc()
        return None


def create_crop_mosaic(image_pil, masks, bbox, output_path="debug_mosaic.jpg"):
    """
    Cria um mosaico com crops das mascarras para decisao do Qwen.
    Args:
        image_pil: Imagem original PIL
        masks: Lista de mascaras (numpy arrays ou Tensors) do SAM
        bbox: [x1, y1, x2, y2]
    Returns:
        Path to saved mosaic
    """
    if np is None:
        return None

    w, h = image_pil.size
    x1, y1, x2, y2 = bbox
    
    # Margin 10%
    bw = x2 - x1
    bh = y2 - y1
    margin_w = int(bw * 0.1)
    margin_h = int(bh * 0.1)
    
    cx1 = max(0, x1 - margin_w)
    cy1 = max(0, y1 - margin_h)
    cx2 = min(w, x2 + margin_w)
    cy2 = min(h, y2 + margin_h)
    
    crop_w = cx2 - cx1
    crop_h = cy2 - cy1
    
    # Base crop
    base_crop = image_pil.crop((cx1, cy1, cx2, cy2))
    
    # Prepare mosaic: 3 side-by-side
    mosaic_w = crop_w * 3
    mosaic_h = crop_h
    mosaic = Image.new("RGB", (mosaic_w, mosaic_h))
    
    draw = ImageDraw.Draw(mosaic)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 40)
    except:
        font = ImageFont.load_default()

    # Process up to 3 masks
    for i in range(min(3, len(masks))):
        # Handle Mask format - official SAM2 returns numpy bool arrays (H, W)
        m = masks[i]
        if hasattr(m, 'cpu'):  # torch tensor
            m = m.cpu().numpy()
        if m.ndim > 2: m = m.squeeze()
        
        # Convert to PIL for processing
        mask_pil = Image.fromarray((m * 255).astype(np.uint8))
        if mask_pil.size != image_pil.size:
             mask_pil = mask_pil.resize(image_pil.size, Image.NEAREST)
             
        # Crop mask
        mask_crop = mask_pil.crop((cx1, cy1, cx2, cy2))
        
        # Overlay on base crop
        # Create colored overlay
        overlay = Image.new("RGBA", base_crop.size, (0, 255, 0, 0)) # Transparent
        # Paste green where mask is white
        # We need a mask for the alpha channel
        green = Image.new("RGBA", base_crop.size, (0, 255, 0, 100)) # Semi-transparent Green
        
        # Composite
        comp = base_crop.convert("RGBA").copy()
        comp.paste(green, (0,0), mask_crop)
        
        # Paste into mosaic
        mosaic.paste(comp.convert("RGB"), (i * crop_w, 0))
        
        # Label
        draw.text((i * crop_w + 10, 10), str(i+1), fill="white", font=font)
        draw.text((i * crop_w + 12, 12), str(i+1), fill="black", font=font) # shadow

    mosaic.save(output_path)
    print(f"Debug Mosaic saved to: {output_path}")
    return output_path


def select_best_mask_by_bbox_iou(masks, bbox, image_size):
    """
    Pick the mask with highest IoU against the detection bounding box.
    Deterministic and fast — replaces VLM-based selection which hallucinates.
    """
    x1, y1, x2, y2 = bbox
    w, h = image_size

    bbox_region = np.zeros((h, w), dtype=bool)
    bbox_region[y1:y2, x1:x2] = True
    bbox_area = int(bbox_region.sum())

    best_idx = 0
    best_iou = -1.0

    for i in range(len(masks)):
        m = masks[i]
        if hasattr(m, 'cpu'):
            m = m.cpu().numpy()
        if m.ndim > 2:
            m = m.squeeze()
        if m.shape[0] != h or m.shape[1] != w:
            m = np.array(
                Image.fromarray((m * 255).astype(np.uint8)).resize((w, h), Image.NEAREST)
            ) > 127
        else:
            m = m > 0.5

        intersection = int((m & bbox_region).sum())
        union = int((m | bbox_region).sum())
        iou = intersection / union if union > 0 else 0.0

        print(f"  Mask {i+1}: area={int(m.sum())}, bbox_overlap={intersection}/{bbox_area} "
              f"({100*intersection/bbox_area:.0f}%), IoU={iou:.3f}")

        if iou > best_iou:
            best_iou = iou
            best_idx = i

    print(f"  -> Best mask by bbox IoU: {best_idx+1} (IoU={best_iou:.3f})")
    return best_idx


def _parse_mask_panel_choice(content: str):
    """
    Extrai escolha 1–3 da resposta do modelo.
    - Com thinking (Qwen3), a resposta final fica após a tag </think>.
    - Usa o último dígito isolado 1–3 no texto analisado: evita erro quando o modelo
      diz que o painel 1 está errado e o certo é o 2, mas o primeiro match seria 1.
    Returns:
        int em 0..2 ou None se não houver dígito válido.
    """
    if not content or not isinstance(content, str):
        return None
    text = content.strip()
    if "</think>" in text:
        text = text.split("</think>", 1)[-1].strip()
    for line in reversed([ln.strip() for ln in text.splitlines() if ln.strip()]):
        if re.fullmatch(r"[1-3]", line):
            return int(line) - 1
    matches = re.findall(r"\b([1-3])\b", text)
    if matches:
        return int(matches[-1]) - 1
    return None


def _sam_prompt_masks(model, img_pil, point_xy, multimask_output=True):
    """Run Ultralytics SAM prompt inference and return masks/scores."""
    img_np = np.array(img_pil.convert("RGB"))
    results = model(
        img_np,
        points=[point_xy],
        labels=[1],
        conf=0.0,
        verbose=False,
    )
    if not results:
        return np.empty((0, img_np.shape[0], img_np.shape[1]), dtype=np.uint8), np.array([])
    r = results[0]
    if r.masks is None or r.masks.data is None or len(r.masks.data) == 0:
        return np.empty((0, img_np.shape[0], img_np.shape[1]), dtype=np.uint8), np.array([])
    masks_np = (r.masks.data > 0).to(dtype=torch.uint8).cpu().numpy()
    if r.boxes is not None and r.boxes.conf is not None and len(r.boxes.conf) == len(masks_np):
        scores_np = r.boxes.conf.cpu().numpy()
    else:
        scores_np = np.ones(len(masks_np), dtype=np.float32)
    return masks_np, scores_np


def _best_mask_from_results(results):
    """Extract best mask and score from Ultralytics Results list."""
    if not results:
        return None, 0.0
    r = results[0]
    if r.masks is None or r.masks.data is None or len(r.masks.data) == 0:
        return None, 0.0
    masks_np = (r.masks.data > 0).to(dtype=torch.uint8).cpu().numpy()
    if r.boxes is not None and r.boxes.conf is not None and len(r.boxes.conf) == len(masks_np):
        scores_np = r.boxes.conf.cpu().numpy()
    else:
        scores_np = np.ones(len(masks_np), dtype=np.float32)
    best_idx = int(np.argmax(scores_np))
    return masks_np[best_idx], float(scores_np[best_idx])


def encode_image_to_base64(image_input) -> str:
    """
    Encodes an image to base64.
    Args:
        image_input: str (filepath) or PIL.Image
    """
    if isinstance(image_input, str) or isinstance(image_input, Path):
        with open(image_input, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")
    elif isinstance(image_input, Image.Image):
        buffered = io.BytesIO()
        # Convert to RGB if needed (e.g. RGBA) to save as JPEG
        if image_input.mode != 'RGB':
             image_input = image_input.convert('RGB')
        image_input.save(buffered, format="JPEG")
        return base64.b64encode(buffered.getvalue()).decode("utf-8")
    else:
        raise ValueError("Unsupported image input type")


def parse_qwen_response(response_text: str, image_size: tuple[int, int] = None) -> list[dict]:
    """
    Extrai BBs no formato nativo do Qwen <ref>...<box> ou JSON format.
    """
    boxes = []
    w, h = image_size if image_size else (1000, 1000)
    
    # Clean up markdown code blocks if present
    clean_text = response_text.replace("```json", "").replace("```", "").strip()
    
    def _safe_confidence(raw_conf) -> float:
        """Best-effort conversion to float confidence in [0, 1]."""
        if raw_conf is None:
            return 1.0
        if isinstance(raw_conf, (int, float)):
            return float(raw_conf)
        s = str(raw_conf).strip().replace("%", "")
        try:
            val = float(s)
            return (val / 100.0) if val > 1.0 else val
        except Exception:
            return 1.0

    # 0. Try JSON Parsing first (since model seems to prefer it now)
    try:
        data = json.loads(clean_text)
        if isinstance(data, list):
            print("DEBUG: Detectado formato JSON.")
            if len(data) == 0:
                print("DEBUG: Lista vazia retornada (objeto não encontrado pelo Qwen).")
                return []
            for item in data:
                if not isinstance(item, dict):
                    continue
                if 'bbox_2d' in item:
                    # JSON bbox_2d is typically [xmin, ymin, xmax, ymax] in PIXELS
                    b = item['bbox_2d']
                    if not isinstance(b, (list, tuple)) or len(b) < 4:
                        print(f"DEBUG: bbox_2d inválido em item JSON: {item}")
                        continue
                    label = item.get('label', 'object')
                    confidence = _safe_confidence(item.get('confidence', 1.0))
                    
                    # Try to get 3 points, fallback to old single point format just in case
                    grasp_points = item.get('grasp_point_2ds', None)
                    if not grasp_points:
                        gp = item.get('grasp_point_2d', None)
                        if gp: grasp_points = [gp]
                    
                    # O prompt já pede no formato [0-1000], portanto os valores já vêm normalizados!
                    # Não devemos dividir pelo tamanho da imagem novamente.
                    xmin, ymin, xmax, ymax = b[0], b[1], b[2], b[3]
                    
                    grasps_1000 = []
                    if grasp_points:
                        for pt in grasp_points:
                            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                                continue
                            gx, gy = pt[0], pt[1]
                            grasps_1000.append([int(float(gx)), int(float(gy))])
                    
                    boxes.append({
                        'label': label,
                        'bbox_1000': [int(float(xmin)), int(float(ymin)), int(float(xmax)), int(float(ymax))],
                        'grasps_1000': grasps_1000,
                        'confidence': confidence
                    })
            if boxes:
                return boxes
    except Exception as e:
        print(f"DEBUG: Falha ao parsear JSON do Qwen: {e}. Trecho: {clean_text[:220]}")

    # regexes
    # 1. Strict: <box>[[...]]</box>
    pattern_strict = r'<ref>(.*?)</ref>\s*<box>\[\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]\]</box>'
    matches = re.findall(pattern_strict, response_text)
    
    # 2. Simple: <box>[...]</box>
    if not matches:
        pattern_simple = r'<ref>(.*?)</ref>\s*<box>\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]</box>'
        matches = re.findall(pattern_simple, response_text)

    # 3. Loose: <box(...) or similar
    matches_loose = []
    if not matches:
        pattern_loose_full = r'<ref>(.*?)</ref>.*?<box.*?[(\[]\s*(\d+)[,\s]+(\d+)[,\s]+(\d+)[,\s]+(\d+)'
        matches_loose = re.findall(pattern_loose_full, response_text, re.DOTALL)

    # Process matches
    all_found = []
    if matches:
        for m in matches:
            # Strict mode [[ ]] -> Assume 0-1000 Normalized as per prompt instructions
            all_found.append({'label': m[0], 'coords': [int(x) for x in m[1:]], 'source': 'strict'})
    elif matches_loose:
        # Loose/Simple mode -> Assume PIXELS relative to image_size (Common fallback behavior)
        for m in matches_loose:
            all_found.append({'label': m[0], 'coords': [int(x) for x in m[1:]], 'source': 'loose'})

    if not all_found:
        return []

    print(f"DEBUG: Encontrados {len(all_found)} boxes candidatos via {all_found[0]['source']}.")

    for item in all_found:
        label = item['label']
        c1, c2, c3, c4 = item['coords']
        source = item['source']
        
        # === Step 1: Detect Scale (Pixel vs 0-1000) ===
        # Strategy Update: 
        # - STRICT ([[ ]]) -> Trust Prompt -> 0-1000.
        # - LOOSE/JSON -> Trust Model Tendency -> PIXELS.
        # - But if a coordinate > 1000, it MUST be pixels (or broken).
        # - If a coordinate > max(w,h), it might be valid pixels (slightly out of bounds) OR normalized? 
        #   (normalized never > 1000).
        
        explicit_pixels = any(c > 1000 for c in [c1, c2, c3, c4])
        use_pixels_logic = (source == 'loose') or explicit_pixels
        
        # === Step 2: Determine Order ===
        # Qwen Standard (Loose): xmin, ymin, xmax, ymax (Natural)
        # Prompted (Strict): ymin, xmin, ymax, xmax (Forced by prompt)
        
        # Calculate Dimensions for both hypotheses
        # A: ymin, xmin, ymax, xmax
        h_a = c3 - c1
        w_a = c4 - c2
        
        # B: xmin, ymin, xmax, ymax
        w_b = c3 - c1
        h_b = c4 - c2
        
        valid_a = h_a > 0 and w_a > 0
        valid_b = h_b > 0 and w_b > 0
        
        final_coords = []
        
        # Decision Logic:
        # 1. If only one is valid, take it.
        # 2. If both valid, check 'source'.
        #    - Strict ([[ ]]): We ASKED for ymin, xmin. Trust A.
        #    - Loose (( )): Model drifted. Trust native B (xmin, ymin).
        
        if valid_a and not valid_b:
             final_coords = [c2, c1, c4, c3] # A
        elif valid_b and not valid_a:
             final_coords = [c1, c2, c3, c4] # B
        elif valid_a and valid_b:
             if source == 'strict':
                 final_coords = [c2, c1, c4, c3] # Expect A
             else:
                 final_coords = [c1, c2, c3, c4] # Expect B (Natural)
        else:
             # Both invalid. Fallback to B (Natural)
             final_coords = [c1, c2, c3, c4]

        # === Step 3: Normalize everything to 0-1000 ===
        xmin, ymin, xmax, ymax = final_coords
        
        if use_pixels_logic:
            # Convert Pixels -> 0-1000
            xmin = (xmin / w) * 1000
            xmax = (xmax / w) * 1000
            ymin = (ymin / h) * 1000
            ymax = (ymax / h) * 1000
            
        boxes.append({
            'label': label.strip(),
            'bbox_1000': [int(xmin), int(ymin), int(xmax), int(ymax)]
        })
    
    return boxes


def draw_result(image_input, boxes: list[dict], output_path: str):
    """
    Desenha as caixas aplicando APENAS a escala 0-1000 -> dimensões reais.
    Args:
        image_input: str (filepath) or PIL.Image
    """
    if isinstance(image_input, str) or isinstance(image_input, Path):
        img = Image.open(image_input)
    elif isinstance(image_input, Image.Image):
        img = image_input.copy()
    else:
        raise ValueError("Invalid image input for drawing")

    draw = ImageDraw.Draw(img)
    w, h = img.size
    print(f"Dimensões reais da imagem: {w}x{h}")

    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 24)
    except IOError:
        font = ImageFont.load_default()

    for box_data in boxes:
        label = box_data['label']
        # Coordenadas na escala 0-1000 [xmin, ymin, xmax, ymax]
        b1000 = box_data['bbox_1000']
        
        print(f"Raw Qwen coords (0-1000) [x,y,x,y]: {b1000}")

        # === A MÁGICA SIMPLES ===
        # Regra de três simples para desnormalizar
        x1 = int((b1000[0] / 1000.0) * w)
        y1 = int((b1000[1] / 1000.0) * h)
        x2 = int((b1000[2] / 1000.0) * w)
        y2 = int((b1000[3] / 1000.0) * h)
        
        # Clamp para garantir que não saia da imagem
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w-1, x2), min(h-1, y2)
        
        final_bbox = [x1, y1, x2, y2]
        print(f"Final Pixel coords: {final_bbox}")

        # Desenha Caixa (Verde grosso)
        color = '#00FF00'
        line_width = 4
        for i in range(line_width):
             draw.rectangle([x1-i, y1-i, x2+i, y2+i], outline=color)
        
        # Desenha Label
        # Compatibilidade com Pillow recente
        if hasattr(draw, 'textbbox'):
             bbox_text = draw.textbbox((0, 0), label, font=font)
             text_w = bbox_text[2] - bbox_text[0]
             text_h = bbox_text[3] - bbox_text[1]
        else:
             text_w, text_h = draw.textsize(label, font=font)

        draw.rectangle([x1, y1 - text_h - 4, x1 + text_w + 4, y1], fill=color)
        draw.text((x1 + 2, y1 - text_h - 2), label, fill='black', font=font)

        if box_data.get('grasps_1000'):
            for g1000 in box_data['grasps_1000']:
                gx = int((g1000[0] / 1000.0) * w)
                gy = int((g1000[1] / 1000.0) * h)
                gx = max(0, min(w-1, gx))
                gy = max(0, min(h-1, gy))
                # Desenha Ponto de Grasp (Vermelho)
                r = 5
                draw.ellipse([gx-r, gy-r, gx+r, gy+r], fill='red')

    if img.mode == 'RGBA':
        img = img.convert('RGB')
    img.save(output_path)
    print(f"\nImagem salva em: {output_path}")


def detect_object(image_input, object_prompt, vllm_url):
    """
    Args:
        image_input: str (filepath) or PIL.Image
    """
    base64_img = encode_image_to_base64(image_input)
    
    # Prompt focado em JSON para extrair Confiança e permitir rejeição
    prompt = f"""Task: Detect '{object_prompt}'.
If the object is clearly visible, return its bounding box, a confidence score (0.0 to 1.0), and exactly 3 distinct 2D grasping points.
The 3 grasp points MUST be physically far apart from each other, representing different valid locations where a robot could grasp the object.
Output STRICTLY in JSON format as a list of dictionaries:
[ {{"label": "{object_prompt}", "bbox_2d": [xmin, ymin, xmax, ymax], "grasp_point_2ds": [[x1, y1], [x2, y2], [x3, y3]], "confidence": 0.95}} ]
If the object is NOT present or mostly occluded, return an empty list: []
Ensure bounding box and grasp point coordinates are normalized to [0-1000] scale."""

    payload = {
        "model": "Qwen/Qwen3-VL-4B-Instruct", 
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_img}"}},
                    {"type": "text", "text": prompt}
                ]
            }
        ],
        "max_tokens": 512,
        "temperature": 0.01 
    }
    
    print("Enviando request pro vLLM...")
    start_time = time.time()
    response = requests.post(f"{vllm_url}/v1/chat/completions", json=payload, timeout=60)
    end_time = time.time()
    
    elapsed_ms = (end_time - start_time) * 1000
    print(f"Request latency: {elapsed_ms:.2f} ms")
    
    response.raise_for_status()
    return response.json()['choices'][0]['message']['content']


def _quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix (numpy)."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])


def rotate_image_upright(img_pil, angle_deg):
    """
    Rotate a PIL image by angle_deg (CCW positive, OpenCV convention).
    Expands canvas so no content is clipped. Fills border with neutral gray.
    Returns (rotated_pil, M_forward, (rot_w, rot_h)).
    """
    img_np = np.array(img_pil.convert("RGB"))
    h, w = img_np.shape[:2]
    center = (w / 2.0, h / 2.0)

    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    rot_w = int(h * sin_a + w * cos_a)
    rot_h = int(h * cos_a + w * sin_a)

    M[0, 2] += (rot_w - w) / 2.0
    M[1, 2] += (rot_h - h) / 2.0

    rotated_np = cv2.warpAffine(
        img_np, M, (rot_w, rot_h), borderValue=(127, 127, 127)
    )
    rotated_pil = Image.fromarray(rotated_np)
    return rotated_pil, M, (rot_w, rot_h)


def inverse_rotate_coords_1000(bbox_1000, grasps_1000, M_forward, rotated_size, original_size):
    """
    Map bbox and grasp points from the rotated image's [0-1000] space
    back to the original image's [0-1000] space.

    Args:
        bbox_1000: [xmin, ymin, xmax, ymax] in [0-1000] of the rotated image
        grasps_1000: list of [x, y] in [0-1000] of the rotated image
        M_forward: 2x3 affine matrix used to produce the rotated image
        rotated_size: (rot_w, rot_h) of the rotated image
        original_size: (orig_w, orig_h) of the original image

    Returns:
        (corrected_bbox_1000, corrected_grasps_1000)
    """
    rot_w, rot_h = rotated_size
    orig_w, orig_h = original_size
    M_inv = cv2.invertAffineTransform(M_forward)

    xmin = bbox_1000[0] / 1000.0 * rot_w
    ymin = bbox_1000[1] / 1000.0 * rot_h
    xmax = bbox_1000[2] / 1000.0 * rot_w
    ymax = bbox_1000[3] / 1000.0 * rot_h

    corners = np.array([
        [xmin, ymin],
        [xmax, ymin],
        [xmax, ymax],
        [xmin, ymax],
    ], dtype=np.float64)

    ones = np.ones((4, 1), dtype=np.float64)
    corners_h = np.hstack([corners, ones])
    orig_corners = (M_inv @ corners_h.T).T  # (4, 2)

    ox_min = max(0.0, np.min(orig_corners[:, 0]))
    oy_min = max(0.0, np.min(orig_corners[:, 1]))
    ox_max = min(float(orig_w), np.max(orig_corners[:, 0]))
    oy_max = min(float(orig_h), np.max(orig_corners[:, 1]))

    corrected_bbox = [
        int(ox_min / orig_w * 1000),
        int(oy_min / orig_h * 1000),
        int(ox_max / orig_w * 1000),
        int(oy_max / orig_h * 1000),
    ]

    corrected_grasps = []
    for g in (grasps_1000 or []):
        gx_px = g[0] / 1000.0 * rot_w
        gy_px = g[1] / 1000.0 * rot_h
        pt_h = np.array([gx_px, gy_px, 1.0])
        orig_pt = M_inv @ pt_h
        ox = max(0.0, min(float(orig_w), orig_pt[0]))
        oy = max(0.0, min(float(orig_h), orig_pt[1]))
        corrected_grasps.append([int(ox / orig_w * 1000), int(oy / orig_h * 1000)])

    return corrected_bbox, corrected_grasps


def _camera_info_distortion_coeffs(msg):
    """Vetor D para cv2.projectPoints; None se sem distorção significativa."""
    if msg is None or not msg.d:
        return None
    d = np.array([float(x) for x in msg.d], dtype=np.float64).reshape(-1, 1)
    if d.size == 0 or float(np.max(np.abs(d))) < 1e-15:
        return None
    return d


def _project_cam_ros_point_to_uv(X, Y, Z, fx, fy, cx, cy, d_coeffs, w: int, h: int):
    """
    Ponto no frame ótico ROS (X→, Y↓, Z frente) → pixel (u,v) float.
    Retorna (u, v, z_ok, in_image) com in_image considerando [0,w) x [0,h).
    """
    if Z <= 1e-5:
        return 0.0, 0.0, False, False
    if d_coeffs is None:
        u = float(fx) * float(X) / float(Z) + float(cx)
        v = float(fy) * float(Y) / float(Z) + float(cy)
    else:
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.zeros(3, dtype=np.float64)
        P = np.array([[X, Y, Z]], dtype=np.float64).reshape(-1, 1, 3)
        img, _ = cv2.projectPoints(P, rvec, tvec, K, d_coeffs)
        u = float(img[0, 0, 0])
        v = float(img[0, 0, 1])
    in_image = (0.0 <= u < float(w)) and (0.0 <= v < float(h))
    return u, v, True, in_image


def _secondary_camera_info_topic(cam_name: str, path_prefix: str) -> str:
    """Monta tópico camera_info: default estilo /frontleft/camera_info ou /camera/frontleft/camera_info."""
    p = (path_prefix or "").strip()
    if not p or p == "/":
        return f"/{cam_name}/camera_info"
    return f"{p.rstrip('/')}/{cam_name}/camera_info"


def _secondary_camera_rgb_topic(cam_name: str, path_prefix: str, rgb_suffix: str) -> str:
    """Monta tópico RGB secundário. Default: /{cam}/rgb ou /camera/{cam}/rgb."""
    p = (path_prefix or "").strip()
    sfx = (rgb_suffix or "/rgb").strip()
    if not sfx.startswith("/"):
        sfx = f"/{sfx}"
    if not p or p == "/":
        return f"/{cam_name}{sfx}"
    return f"{p.rstrip('/')}/{cam_name}{sfx}"


# =============================================================================
# ROS 2 NODE: DetectQwenNode
# =============================================================================

class DetectQwenNode(Node):
    """
    ROS 2 Node that detects objects using Qwen + SAM2 VideoPredictor and
    publishes the target's 3D pose as a TF (odom → target_object).
    
    Strategy:
      1. Initial detection: Qwen detects bbox → SAM2 VideoPredictor conditions on frame 0
      2. Tracking: Each new camera frame is propagated via SAM2 (~30ms)
      3. Re-detection: Only triggered if tracking confidence drops or object is lost
    """
    
    # Safety reset window
    # 300 frames @ 5Hz = ~1 minute.
    MAX_TRACKING_FRAMES = 300
    
    def __init__(self):
        super().__init__('detect_qwen_node')
        
        # Parameters
        self.declare_parameter('object_prompt', 'wheel valve')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('target_frame', 'target_object')
        # TF parent for target pose: Spot driver (BD) publishes "base" -> odom; spot_description URDF uses "body"/"base_link".
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('rgb_topic', '/hand/rgb')
        self.declare_parameter('depth_topic', '/hand/depth')
        self.declare_parameter('camera_info_topic', '/hand/camera_info')
        self.declare_parameter('depth_info_topic', '/hand/camera_info')
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_slop_sec', 0.25)
        self.declare_parameter('visualize', True)
        self.declare_parameter('tracking_window_name', 'SAM2 Live Tracking')
        
        self.object_prompt = self.get_parameter('object_prompt').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_frame_name = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.visualize = self.get_parameter('visualize').value
        self.tracking_window_name = self.get_parameter('tracking_window_name').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        self.sync_queue_size = int(max(5, self.get_parameter('sync_queue_size').value))
        self.sync_slop_sec = float(max(0.05, self.get_parameter('sync_slop_sec').value))
        
        self.get_logger().info(f"Object prompt: '{self.object_prompt}'")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"Target frame: {self.target_frame_name}")
        self.get_logger().info(f"Robot base frame (TF): {self.robot_base_frame}")
        
        # State
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None   # (fx, fy, cx, cy)
        self.camera_frame_id = None     # frame_id from CameraInfo
        self.target_transform = None    # TransformStamped (odom → target)
        self.detection_running = False
        self.initial_detection_done = False
        
        # Tracking state
        self.declare_parameter('depth_segmented_topic', '/hand/depth_segmented')
        depth_segmented_topic = self.get_parameter('depth_segmented_topic').value
        self.depth_seg_pub = self.create_publisher(RosImage, depth_segmented_topic, 10)

        self.declare_parameter('segmentation_mask_topic', '/hand/segmentation_mask')
        self.declare_parameter('publish_segmentation_mask', True)
        mask_topic = self.get_parameter('segmentation_mask_topic').value
        self.publish_segmentation_mask = self.get_parameter('publish_segmentation_mask').value
        self.mask_pub = self.create_publisher(RosImage, mask_topic, 10)
        
        self.video_predictor = None
        self.tracking_active = False
        self.new_frame_available = False
        self._node_start_wall = time.time()
        self._last_synced_pair_wall = None
        self._last_no_sync_warn_wall = 0.0
        self._rgb_tap_count = 0
        self._depth_tap_count = 0
        self._last_rgb_stamp_sec = None
        self._last_depth_stamp_sec = None
        self._latest_rgb_msg = None
        self._latest_depth_msg = None
        self._last_pair_key = None
        self._synced_pair_count = 0
        self._last_stale_frame_warn_wall = 0.0
        self._last_depth_stamp_ns = None
        self._last_ros_now_ns = None
        self.tracking_frame_count = 0
        self.last_tracking_score = 0.0
        
        # Tracking runtime parameters
        self.declare_parameter('active_tracking_interval', 0.2)
        self.declare_parameter('gpu_yield_after_track', True)
        self.declare_parameter('tracking_confidence_threshold', 0.15)
        self.declare_parameter('tracking_min_mask_area', 1200)
        self.declare_parameter('tracking_lost_confirm_frames', 5)
        self.declare_parameter('tracking_redetect_cooldown_sec', 2.5)
        
        self.active_tracking_interval = self.get_parameter('active_tracking_interval').value
        self.gpu_yield = self.get_parameter('gpu_yield_after_track').value
        self.tracking_confidence_threshold = float(
            self.get_parameter('tracking_confidence_threshold').value
        )
        self.tracking_min_mask_area = int(
            self.get_parameter('tracking_min_mask_area').value
        )
        self.tracking_lost_confirm_frames = int(
            max(1, self.get_parameter('tracking_lost_confirm_frames').value)
        )
        self.tracking_redetect_cooldown_sec = float(
            max(0.0, self.get_parameter('tracking_redetect_cooldown_sec').value)
        )
        
        self._last_track_time = 0.0
        self._tracking_lost_streak = 0
        self._last_redetect_time = 0.0
        self._adaptive_tracking_min_mask_area = None
        self._target_pose_measure_wall_time = None
        self._target_pose_seq = 0
        self._odom_reject_streak = 0
        self._last_hand_tf_diag_wall = 0.0
        
        # Trigger FOV câmeras secundárias (frontais): só logs nesta fase
        self.declare_parameter('enable_secondary_fov_trigger', True)
        self.declare_parameter('secondary_camera_names', 'frontleft,frontright')
        self.declare_parameter('secondary_camera_path_prefix', '')
        self.declare_parameter('odom_consistency_frame', 'odom')
        self.declare_parameter('max_target_pose_age_sec', 0.75)
        self.declare_parameter('secondary_trigger_max_pose_age_sec', 0.45)
        self.declare_parameter('max_consistency_position_jump_m', 0.35)
        self.declare_parameter('odom_reject_confirm_ticks', 3)
        self.declare_parameter('consistency_ema_alpha', 0.25)
        self.declare_parameter('fov_trigger_confirm_ticks', 2)
        self.declare_parameter('fov_release_confirm_ticks', 2)
        self.declare_parameter('secondary_fov_tf_lookup_timeout_sec', 0.2)
        self.declare_parameter('secondary_fov_ignore_distortion', False)
        self.declare_parameter('hand_tf_lookup_timeout_sec', 1.0)
        # Ignore sensor-stamp lookups when frame is too old (prevents stale-frame TF extrapolation).
        self.declare_parameter('max_sensor_age_for_tf_sec', 1.2)
        # Small tolerance for "future extrapolation" jitter; allows bounded fallback to latest TF.
        self.declare_parameter('tf_future_tolerance_sec', 0.35)
        # Histórico TF no nó (s). O default do tf2 (10s) apaga transforms em playback com saltos de tempo.
        self.declare_parameter('tf_buffer_cache_time_sec', 120.0)
        # False: lookup com stamp do depth / ROS time alinhado à imagem (real + sim em movimento).
        # True: lookup em "agora" sem bloquear (rápido; pode misturar pose com frame atrasado).
        self.declare_parameter('use_wall_time_tf_lookups', False)
        self.declare_parameter('show_secondary_debug_windows', True)
        self.declare_parameter('secondary_camera_rgb_suffix', '/rgb')
        self.declare_parameter('enable_secondary_video_tracking', True)
        self.declare_parameter('secondary_tracking_interval', 0.2)
        self.declare_parameter('secondary_tracking_priority', '')
        self.declare_parameter('resume_hand_after_secondary_stop', 'none')

        self.enable_secondary_fov_trigger = bool(
            self.get_parameter('enable_secondary_fov_trigger').value
        )
        sec_names_raw = self.get_parameter('secondary_camera_names').value or ''
        self._secondary_camera_names = [
            x.strip() for x in str(sec_names_raw).split(',') if x.strip()
        ]
        self.secondary_camera_path_prefix = str(
            self.get_parameter('secondary_camera_path_prefix').value or ''
        )
        self.odom_consistency_frame = str(
            self.get_parameter('odom_consistency_frame').value or 'odom'
        )
        self.declare_parameter('target_parent_frame', '')
        target_parent_frame_raw = str(self.get_parameter('target_parent_frame').value or '').strip()
        self.target_parent_frame = target_parent_frame_raw or self.odom_consistency_frame
        # Correção de roll (Qwen): precisa de TF reference_frame -> câmera. Em sim, hand_cam muitas vezes
        # liga a odom/world mas não a `robot_base_frame` (árvores desligadas). Default = mesmo parent do alvo.
        self.declare_parameter('roll_correction_reference_frame', '')
        _roll_ref = str(
            self.get_parameter('roll_correction_reference_frame').value or ''
        ).strip()
        self.roll_correction_reference_frame = _roll_ref or self.target_parent_frame
        self.max_target_pose_age_sec = float(
            self.get_parameter('max_target_pose_age_sec').value
        )
        self.secondary_trigger_max_pose_age_sec = float(
            self.get_parameter('secondary_trigger_max_pose_age_sec').value
        )
        self.max_consistency_position_jump_m = float(
            self.get_parameter('max_consistency_position_jump_m').value
        )
        self.odom_reject_confirm_ticks = int(
            max(1, self.get_parameter('odom_reject_confirm_ticks').value)
        )
        self.consistency_ema_alpha = float(
            min(1.0, max(0.01, self.get_parameter('consistency_ema_alpha').value))
        )
        self.fov_trigger_confirm_ticks = int(
            max(1, self.get_parameter('fov_trigger_confirm_ticks').value)
        )
        self.fov_release_confirm_ticks = int(
            max(1, self.get_parameter('fov_release_confirm_ticks').value)
        )
        _sec_fov_to = float(self.get_parameter('secondary_fov_tf_lookup_timeout_sec').value)
        _MIN_TF_TIMEOUT = 0.05
        if _sec_fov_to < _MIN_TF_TIMEOUT:
            self.get_logger().info(
                f"secondary_fov_tf_lookup_timeout_sec={_sec_fov_to} < {_MIN_TF_TIMEOUT}; "
                f"a usar {_MIN_TF_TIMEOUT} s (evita falhas com timeout 0)."
            )
        self.secondary_fov_tf_lookup_timeout_sec = max(_MIN_TF_TIMEOUT, max(0.0, _sec_fov_to))
        self.secondary_fov_ignore_distortion = bool(
            self.get_parameter('secondary_fov_ignore_distortion').value
        )
        _hand_to = float(max(0.0, self.get_parameter('hand_tf_lookup_timeout_sec').value))
        if _hand_to < _MIN_TF_TIMEOUT:
            self.get_logger().info(
                f"hand_tf_lookup_timeout_sec={_hand_to} < {_MIN_TF_TIMEOUT}; "
                f"a usar {_MIN_TF_TIMEOUT} s (timeout 0 impede espera pelo buffer TF no arranque)."
            )
        self.hand_tf_lookup_timeout_sec = max(_MIN_TF_TIMEOUT, _hand_to)
        self.max_sensor_age_for_tf_sec = float(
            max(0.05, self.get_parameter('max_sensor_age_for_tf_sec').value)
        )
        self.tf_future_tolerance_sec = float(
            max(0.0, self.get_parameter('tf_future_tolerance_sec').value)
        )
        _tf_cache = float(self.get_parameter('tf_buffer_cache_time_sec').value)
        self.tf_buffer_cache_time_sec = max(5.0, _tf_cache)
        self.use_wall_time_tf_lookups = bool(
            self.get_parameter('use_wall_time_tf_lookups').value
        )
        self.show_secondary_debug_windows = bool(
            self.get_parameter('show_secondary_debug_windows').value
        )
        self.secondary_camera_rgb_suffix = str(
            self.get_parameter('secondary_camera_rgb_suffix').value or '/rgb'
        )
        self.enable_secondary_video_tracking = bool(
            self.get_parameter('enable_secondary_video_tracking').value
        )
        self.secondary_tracking_interval = float(
            max(0.05, self.get_parameter('secondary_tracking_interval').value)
        )
        _prio_raw = self.get_parameter('secondary_tracking_priority').value or ''
        self._secondary_tracking_priority = [
            x.strip() for x in str(_prio_raw).split(',') if x.strip()
        ]
        self.resume_hand_after_secondary_stop = str(
            self.get_parameter('resume_hand_after_secondary_stop').value or 'none'
        ).lower()

        self._gpu_lock = threading.Lock()
        self._target_tf_wall_time = None
        self._consistency_ref_ema = None
        self._secondary_cam_state = {}
        for name in self._secondary_camera_names:
            self._secondary_cam_state[name] = {
                'info_msg': None,
                'intrinsics': None,
                'frame_id': None,
                'width': 0,
                'height': 0,
                'd_coeffs': None,
                'trigger_active': False,
                'prev_trigger': False,
                'in_view_streak': 0,
                'out_streak': 0,
                'latest_rgb_bgr': None,
                'frame_pending': False,
                'video_predictor': None,
                'pending_secondary_init': False,
                'init_uv': None,
                'tracking_initialized': False,
                'last_sec_track_time': 0.0,
                'sec_frame_count': 0,
                'last_mask': None,
                'last_score': 0.0,
                'last_projected_uv': None,
            }
        
        # TF2 (cache longo ajuda simulação/playback quando o relógio salta ou há filas)
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=self.tf_buffer_cache_time_sec)
        )
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            f"TF buffer cache_time={self.tf_buffer_cache_time_sec:.1f}s "
            f"(param tf_buffer_cache_time_sec); hand lookup timeout={self.hand_tf_lookup_timeout_sec:.2f}s; "
            f"tf_listener_spin_thread=False"
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Camera info subscriber (one-shot, just need intrinsics)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            depth_info_topic,
            self._camera_info_cb,
            1
        )

        self._secondary_info_subs = []
        self._secondary_rgb_subs = []
        if self.enable_secondary_fov_trigger and self._secondary_camera_names:
            for cam_name in self._secondary_camera_names:
                ctopic = _secondary_camera_info_topic(
                    cam_name, self.secondary_camera_path_prefix
                )
                sub = self.create_subscription(
                    CameraInfo,
                    ctopic,
                    lambda msg, name=cam_name: self._secondary_camera_info_cb(msg, name),
                    1,
                )
                self._secondary_info_subs.append(sub)
                _need_rgb = self.enable_secondary_video_tracking or (
                    self.show_secondary_debug_windows and self.visualize
                )
                if _need_rgb:
                    rtopic = _secondary_camera_rgb_topic(
                        cam_name,
                        self.secondary_camera_path_prefix,
                        self.secondary_camera_rgb_suffix,
                    )
                    rgb_sub = self.create_subscription(
                        RosImage,
                        rtopic,
                        lambda msg, name=cam_name: self._secondary_rgb_cb(msg, name),
                        1,
                    )
                    self._secondary_rgb_subs.append(rgb_sub)
            self.get_logger().info(
                "FOV trigger secundário: "
                f"câmeras={self._secondary_camera_names}, "
                f"frame consistência={self.odom_consistency_frame}, "
                f"jump_max={self.max_consistency_position_jump_m}m"
            )
        
        # Synced RGB + Depth subscribers (ApproximateTimeSynchronizer).
        self._image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, self.sync_queue_size),
        )
        self.rgb_sub = message_filters.Subscriber(
            self, RosImage, rgb_topic, qos_profile=self._image_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, RosImage, depth_topic, qos_profile=self._image_qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )
        self.sync.registerCallback(self._synced_image_cb)
        # Taps para diagnóstico (não participam do pareamento principal).
        self.rgb_tap_sub = self.create_subscription(RosImage, rgb_topic, self._rgb_tap_cb, 10)
        self.depth_tap_sub = self.create_subscription(RosImage, depth_topic, self._depth_tap_cb, 10)
        self.get_logger().info(
            f"RGB/Depth sync config (ATS): "
            f"queue_size={self.sync_queue_size}, slop={self.sync_slop_sec:.3f}s, "
            "reliability=BEST_EFFORT"
        )
        
        # Tracking timer (10 Hz poll — actual rate controlled adaptively inside callback)
        
        # Isolate heavy inference tracking step so it doesn't block TF listener
        self.inference_cb_group = MutuallyExclusiveCallbackGroup()
        self.tracking_timer = self.create_timer(
            0.1, self._tracking_timer_cb, callback_group=self.inference_cb_group
        )
        
        # TF publish timer (10 Hz)
        self.tf_timer = self.create_timer(0.1, self._tf_publish_cb)
        
        # Pre-load SAM2 VideoPredictor
        self.get_logger().info("Pre-loading SAM2 VideoPredictor...")
        self.video_predictor = load_sam_video_model()
        if self.video_predictor is None:
            self.get_logger().error("Failed to load SAM2 VideoPredictor!")
        else:
            self.get_logger().info("SAM2 VideoPredictor loaded.")
        
        self.get_logger().info(
            f"Segmentation mask topic: {mask_topic} (publish={self.publish_segmentation_mask})"
        )
        self.get_logger().info(
            f"Roll correction TF: {self.roll_correction_reference_frame!r} "
            f"(param roll_correction_reference_frame; vazio => target_parent_frame)"
        )

        self.get_logger().info("DetectQwenNode ready. Waiting for camera data...")
    
    def _camera_info_cb(self, msg: CameraInfo):
        """Extract camera intrinsics once."""
        if self.camera_intrinsics is None:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.camera_intrinsics = (fx, fy, cx, cy)
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info(
                f"Camera intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
            )
            self.get_logger().info(f"Camera frame_id: '{self.camera_frame_id}'")

    def _explain_hand_tf_stamp_failure(
        self,
        target_frame: str,
        source_frame: str,
        sensor_header,
        exc: Exception,
    ):
        """Explica falhas de lookup no tempo do sensor (throttle; não esconde a causa)."""
        now_wall = time.time()
        if now_wall - self._last_hand_tf_diag_wall < 5.0:
            return
        self._last_hand_tf_diag_wall = now_wall
        raw = str(exc)
        m = re.search(
            r"Requested time ([\d.]+) but the earliest data is at time ([\d.]+)",
            raw,
        )
        stamp_msg_sec = None
        if sensor_header is not None:
            stamp_msg_sec = (
                float(sensor_header.stamp.sec)
                + float(sensor_header.stamp.nanosec) * 1e-9
            )
        now_ros = self.get_clock().now()
        now_ros_sec = float(now_ros.nanoseconds) * 1e-9
        parts = [
            "[TF+sensor] lookup falhou com o stamp do header da mensagem (sem fallback silencioso).",
            f"  Pedido: transformação {source_frame!r} → {target_frame!r} no tempo do depth/RGB.",
        ]
        if stamp_msg_sec is not None:
            parts.append(f"  stamp no Header da mensagem: {stamp_msg_sec:.6f} s (domínio ROS do publicador).")
        parts.append(f"  relógio atual do nó detect_qwen: {now_ros_sec:.6f} s.")
        if m:
            req_t = float(m.group(1))
            early_t = float(m.group(2))
            gap = early_t - req_t
            parts.append(
                f"  tf2 diz: pediste t={req_t:.6f}s, mas o buffer só tem dados desta cadeia a partir de t={early_t:.6f}s "
                f"(o teu frame é ~{gap:.3f}s **mais antigo** que o histórico disponível)."
            )
            parts.append(
                "  Interpretação: não é ‘só interpolação’ — o instante do depth está **antes** de qualquer transform "
                "guardada para essa ligação. Causas típicas: (1) fila/atraso e estás a processar um frame velho enquanto "
                "o TF já avançou ou foi reiniciado; (2) câmera e TF não partilham o mesmo relógio (/clock vs wall, ou "
                "nós com use_sim_time incoerente); (3) static_transform / driver do braço publicado só depois do sim "
                "já ter avançado. Corrigir na fonte (stamps e /clock), não mascarar com lookup em ‘agora’ sem saber."
            )
        elif re.search(r"extrapolation into the future", raw, re.I):
            parts.append(
                "  tf2: extrapolação para o **futuro** — o stamp do depth/imagem está **à frente** do último transform "
                "conhecido nessa cadeia (ou à frente do /clock). Mesmo com ‘playback tick’, pode acontecer se a câmera "
                "publicar antes do tf_static/dynamic desse instante, ou se o buffer perdeu histórico (aumenta "
                "tf_buffer_cache_time_sec). Alternativa explícita: use_wall_time_tf_lookups:=true."
            )
        else:
            parts.append(
                "  Outro tipo de falha TF; vê a exceção abaixo. Verifica fixed frame, nomes de frames e conectividade."
            )
        parts.append(f"  Exceção: {raw}")
        self.get_logger().error("\n".join(parts))

    def _wait_hand_tf_chain_ready(self, max_wall_sec: float = 5.0) -> bool:
        """
        No arranque, imagem pode chegar antes do grafo TF.
        Só exige o caminho usado em _mask_to_tf: target_parent_frame <- câmera.
        (robot_base_frame pode estar noutra árvore que não inclui hand_cam — típico em sim.)
        """
        dh = getattr(self, "latest_depth_header", None)
        source = getattr(self, "camera_frame_id", None) or (
            dh.frame_id if dh is not None else "hand_cam"
        )
        target = self.target_parent_frame
        deadline = time.time() + max(0.1, max_wall_sec)
        step = Duration(seconds=self.hand_tf_lookup_timeout_sec)
        ros_now = rclpy.time.Time()
        t_wait0 = time.time()
        warned = False
        while time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(target, source, ros_now, timeout=step)
                return True
            except Exception:
                pass
            if not warned and (time.time() - t_wait0) > 0.5:
                warned = True
                self.get_logger().info(
                    f"A aguardar TF {source!r} -> {target!r} (até {max_wall_sec:.1f}s)…"
                )
            time.sleep(0.02)
        return False

    def _lookup_transform_with_sensor_stamp(
        self, target_frame: str, source_frame: str, sensor_header=None
    ):
        """
        Com use_wall_time_tf_lookups: lookup em rclpy.time.Time() (instante do nó).

        Caso contrário: **só** lookup no stamp do header do sensor — se falhar, regista diagnóstico e propaga
        (sem fallback para Time(0)/agora, para não esconder desincronização).
        """
        timeout = Duration(seconds=self.hand_tf_lookup_timeout_sec)
        if self.use_wall_time_tf_lookups:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.hand_tf_lookup_timeout_sec),
            )
        if sensor_header is None:
            err = RuntimeError(
                "use_wall_time_tf_lookups=false requer header do depth/câmera; "
                "latest_depth_header não está definido."
            )
            self._explain_hand_tf_stamp_failure(
                target_frame, source_frame, sensor_header, err
            )
            raise err
        st = rclpy.time.Time.from_msg(sensor_header.stamp)
        # #region agent log
        try:
            now_ros_sec = float(self.get_clock().now().nanoseconds) * 1e-9
            sensor_sec = float(sensor_header.stamp.sec) + float(sensor_header.stamp.nanosec) * 1e-9
            _append_debug_log(
                "H6",
                "detect_qwen.py:_lookup_transform_with_sensor_stamp:before_lookup",
                "sensor-stamp tf lookup attempt",
                {
                    "target_frame": str(target_frame),
                    "source_frame": str(source_frame),
                    "sensor_stamp_sec": sensor_sec,
                    "node_now_ros_sec": now_ros_sec,
                    "sensor_age_vs_node_now_sec": now_ros_sec - sensor_sec,
                    "timeout_sec": float(self.hand_tf_lookup_timeout_sec),
                    "use_wall_time_tf_lookups": bool(self.use_wall_time_tf_lookups),
                },
            )
        except Exception:
            pass
        # #endregion
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, st, timeout=timeout
            )
        except Exception as e:
            # #region agent log
            raw = str(e)
            req_t = None
            latest_t = None
            earliest_t = None
            try:
                m_future = re.search(
                    r"Requested time ([\d.]+) but the latest data is at time ([\d.]+)",
                    raw,
                )
                m_past = re.search(
                    r"Requested time ([\d.]+) but the earliest data is at time ([\d.]+)",
                    raw,
                )
                if m_future:
                    req_t = float(m_future.group(1))
                    latest_t = float(m_future.group(2))
                if m_past:
                    req_t = float(m_past.group(1))
                    earliest_t = float(m_past.group(2))
                _append_debug_log(
                    "H7",
                    "detect_qwen.py:_lookup_transform_with_sensor_stamp:lookup_error",
                    "sensor-stamp tf lookup failure",
                    {
                        "target_frame": str(target_frame),
                        "source_frame": str(source_frame),
                        "exception": raw[:500],
                        "requested_time_sec": req_t,
                        "latest_time_sec": latest_t,
                        "earliest_time_sec": earliest_t,
                        "future_delta_sec": (req_t - latest_t) if (req_t is not None and latest_t is not None) else None,
                        "past_delta_sec": (earliest_t - req_t) if (req_t is not None and earliest_t is not None) else None,
                    },
                )
            except Exception:
                pass
            # #endregion
            # Bounded fallback: if TF is only slightly behind sensor stamp, use latest available TF.
            if latest_t is not None and req_t is not None:
                future_delta = req_t - latest_t
                if 0.0 <= future_delta <= self.tf_future_tolerance_sec:
                    try:
                        latest_sec = int(latest_t)
                        latest_nsec = int((latest_t - latest_sec) * 1e9)
                        t_latest = self.tf_buffer.lookup_transform(
                            target_frame,
                            source_frame,
                            rclpy.time.Time(seconds=latest_sec, nanoseconds=latest_nsec),
                            timeout=timeout,
                        )
                        # #region agent log
                        try:
                            _append_debug_log(
                                "H8",
                                "detect_qwen.py:_lookup_transform_with_sensor_stamp:bounded_fallback_success",
                                "used bounded fallback to latest tf time",
                                {
                                    "target_frame": str(target_frame),
                                    "source_frame": str(source_frame),
                                    "future_delta_sec": float(future_delta),
                                    "tolerance_sec": float(self.tf_future_tolerance_sec),
                                    "latest_time_sec": float(latest_t),
                                },
                            )
                        except Exception:
                            pass
                        # #endregion
                        return t_latest
                    except Exception as e_fb:
                        # #region agent log
                        try:
                            _append_debug_log(
                                "H8",
                                "detect_qwen.py:_lookup_transform_with_sensor_stamp:bounded_fallback_failed",
                                "bounded fallback to latest tf time failed",
                                {
                                    "target_frame": str(target_frame),
                                    "source_frame": str(source_frame),
                                    "future_delta_sec": float(future_delta),
                                    "tolerance_sec": float(self.tf_future_tolerance_sec),
                                    "fallback_exception": str(e_fb)[:300],
                                },
                            )
                        except Exception:
                            pass
                        # #endregion
            self._explain_hand_tf_stamp_failure(
                target_frame, source_frame, sensor_header, e
            )
            raise

    def _get_correction_angle(self, depth_header=None):
        """
        Ângulo de roll (graus) para endireitar a imagem no Qwen: projeta gravidade no plano da imagem
        via TF roll_correction_reference_frame (default = target_parent_frame, ex. odom), não robot_base_frame,
        para não exigir que hand_cam esteja na mesma árvore que `base`.

        Args:
            depth_header: snapshot of depth header to use for TF lookup (avoids desync
                          when self.latest_depth_header is overwritten during inference).
        """
        sensor_h = depth_header if depth_header is not None else getattr(self, "latest_depth_header", None)
        source_frame = getattr(self, 'camera_frame_id', None)
        if not source_frame:
            source_frame = (
                sensor_h.frame_id
                if sensor_h is not None
                else 'hand_cam'
            )

        ref = self.roll_correction_reference_frame
        try:
            t = self._lookup_transform_with_sensor_stamp(
                ref, source_frame, sensor_h
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed for correction angle ({ref!r} <- {source_frame!r}): {e}"
            )
            return 0.0

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        R_body_cam = _quat_to_rotation_matrix(qx, qy, qz, qw)
        R_cam_body = R_body_cam.T

        gravity_body = np.array([0.0, 0.0, -1.0])
        gravity_cam = R_cam_body @ gravity_body

        gx, gy = gravity_cam[0], gravity_cam[1]
        angle_deg = 90.0 - np.degrees(np.arctan2(gy, gx))

        # Normalize to [-180, 180]
        angle_deg = (angle_deg + 180.0) % 360.0 - 180.0

        if abs(angle_deg) < 5.0:
            return 0.0

        return angle_deg

    def _synced_image_cb(self, rgb_msg: RosImage, depth_msg: RosImage):
        """Store latest synced RGB + Depth pair."""
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            self.latest_rgb = Image.fromarray(cv_rgb)
            
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            self.latest_depth_header = depth_msg.header
            self.new_frame_available = True
            self._last_synced_pair_wall = time.time()
            self._synced_pair_count += 1

            
            # Trigger initial detection on first image
            if not self.initial_detection_done:
                self.initial_detection_done = True
                self.get_logger().info("First image received, triggering initial detection...")
                self._run_initial_detection()
                
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def _stamp_sec(self, msg: RosImage) -> float:
        st = msg.header.stamp
        return float(st.sec) + float(st.nanosec) * 1e-9

    def _rgb_tap_cb(self, msg: RosImage):
        self._rgb_tap_count += 1
        self._last_rgb_stamp_sec = self._stamp_sec(msg)
        self._latest_rgb_msg = msg

    def _depth_tap_cb(self, msg: RosImage):
        self._depth_tap_count += 1
        self._last_depth_stamp_sec = self._stamp_sec(msg)
        self._latest_depth_msg = msg
    
    # -----------------------------------------------------------------
    # INITIAL DETECTION (Qwen + SAM2 conditioning)
    # -----------------------------------------------------------------
    # -----------------------------------------------------------------
    # INITIAL DETECTION (Qwen + SAM2 conditioning with Mask Selection)
    # -----------------------------------------------------------------
    def _run_initial_detection(self):
        """Run Qwen to detect, select best mask via Mosaic, then init SAM2 VideoPredictor."""
        if self.detection_running or self.video_predictor is None:
            return
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if self.camera_intrinsics is None:
            self.get_logger().warn("Waiting for camera_info...")
            return
        
        self.detection_running = True
        self.new_frame_available = False
        t_start = time.time()
        # Atomic snapshot: freeze the depth header alongside image data so TF lookups
        # use the stamp matching the actual frame being processed (not a later overwrite).
        snapshot_header = getattr(self, 'latest_depth_header', None)
        # Pre-lookup TF while sensor stamp is still fresh (~0.2s old).
        # After Qwen + SAM2 inference (2-4s), the stamp would be stale.
        cached_transform = None
        try:
            source_frame = getattr(self, 'camera_frame_id', None)
            if not source_frame:
                source_frame = snapshot_header.frame_id if snapshot_header is not None else 'hand_cam'
            cached_transform = self._lookup_transform_with_sensor_stamp(
                self.target_parent_frame, source_frame, snapshot_header
            )
        except Exception as e:
            self.get_logger().warn(
                f"Pre-lookup TF falhou (será tentado novamente em _mask_to_tf): {e}",
                throttle_duration_sec=5.0,
            )
        self.get_logger().info(f"🔍 Running initial detection for '{self.object_prompt}'...")
        if not self._wait_hand_tf_chain_ready(max_wall_sec=5.0):
            self.get_logger().warn(
                f"TF (câmera -> {self.target_parent_frame!r}) não ficou disponível em 5s; "
                "Qwen/_mask_to_tf podem falhar até existir essa ligação no grafo."
            )
        
        try:
            img_pil = self.latest_rgb
            orig_w, orig_h = img_pil.size
            
            # Resize for Qwen
            max_dim = 1024
            scale_factor = max_dim / max(orig_w, orig_h)
            new_w = int(orig_w * scale_factor)
            new_h = int(orig_h * scale_factor)
            img_resized = img_pil.resize((new_w, new_h), Image.LANCZOS)
            
            # Correct camera roll before sending to Qwen
            correction_angle = self._get_correction_angle(depth_header=snapshot_header)
            M_fwd = None
            if abs(correction_angle) > 5.0:
                img_for_qwen, M_fwd, rot_size = rotate_image_upright(img_resized, correction_angle)
                self.get_logger().info(
                    f"Rotated image by {correction_angle:.1f} deg for Qwen "
                    f"({new_w}x{new_h} -> {rot_size[0]}x{rot_size[1]})"
                )
            else:
                img_for_qwen = img_resized
            
            # Step 1: Qwen detection (on upright image)
            t0 = time.time()
            response_text = detect_object(img_for_qwen, self.object_prompt, VLLM_URL)
            t1 = time.time()
            qwen_ms = (t1 - t0) * 1000
            self.get_logger().info(f"Qwen response: {response_text[:60]}...")
            
            qwen_w, qwen_h = img_for_qwen.size
            boxes = parse_qwen_response(response_text, image_size=(qwen_w, qwen_h))
            
            # Map bbox/grasps back to the original (non-rotated) image space
            if M_fwd is not None and boxes:
                for box_data in boxes:
                    box_data['bbox_1000'], box_data['grasps_1000'] = inverse_rotate_coords_1000(
                        box_data['bbox_1000'],
                        box_data.get('grasps_1000', []),
                        M_fwd,
                        (qwen_w, qwen_h),
                        (new_w, new_h),
                    )
                self.get_logger().info("Inverse-rotated bbox/grasps back to original image space")
            if not boxes:
                self.get_logger().warn("No bounding box found (Qwen rejected or no object).")
                self.detection_running = False
                return
            
            # Extract box data
            box_data = boxes[0]
            label = box_data['label']
            b1000 = box_data['bbox_1000']
            qwen_conf = box_data.get('confidence', 1.0)
            g1000_list = box_data.get('grasps_1000', [])
            
            if qwen_conf < self.confidence_threshold:
                self.get_logger().warn(f"Qwen confidence ({qwen_conf:.2f}) < threshold ({self.confidence_threshold}). Rejecting.")
                self.detection_running = False
                return
                
            draw_result(img_pil, boxes, "camera_capture_detected.jpg")
            
            x1 = int((b1000[0] / 1000.0) * orig_w)
            y1 = int((b1000[1] / 1000.0) * orig_h)
            x2 = int((b1000[2] / 1000.0) * orig_w)
            y2 = int((b1000[3] / 1000.0) * orig_h)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(orig_w - 1, x2), min(orig_h - 1, y2)
            bbox = [x1, y1, x2, y2]
            
            grasp_points_uv = []
            if g1000_list:
                for g1000 in g1000_list:
                    gu = int((g1000[0] / 1000.0) * orig_w)
                    gv = int((g1000[1] / 1000.0) * orig_h)
                    gu, gv = max(0, min(orig_w - 1, gu)), max(0, min(orig_h - 1, gv))
                    grasp_points_uv.append([gu, gv])
            else:
                grasp_points_uv = [[(x1+x2)//2, (y1+y2)//2]]
                
            # Keep the primary grasp for offset calculation downstream
            grasp_u, grasp_v = grasp_points_uv[0]
            self.grasp_points_uv_stored = grasp_points_uv  # Store for depth fallback
            
            # Step 2: Generate Candidate Masks & Select Best
            t2 = time.time()
            self.get_logger().info("Generating candidate masks using SAM2 based on Qwen points...")
            
            sam_img_predictor = load_sam_model()
            if sam_img_predictor:
                all_masks = []
                for pt in grasp_points_uv:
                    masks, scores = _sam_prompt_masks(
                        sam_img_predictor, img_pil, pt, multimask_output=True
                    )
                    if len(masks) == 0:
                        continue
                    best_local_idx = int(np.argmax(scores))
                    all_masks.append(masks[best_local_idx])

                best_idx = 0
                if len(all_masks) > 1:
                    create_crop_mosaic(img_pil, all_masks, bbox, "debug_mosaic.jpg")
                    self.get_logger().info("Selecting best mask by bbox IoU...")
                    best_idx = select_best_mask_by_bbox_iou(all_masks, bbox, img_pil.size)
                    self.get_logger().info(f"Selected mask {best_idx+1} by bbox IoU")

                best_mask = all_masks[best_idx] if all_masks else None
                if best_mask is not None:
                    grasp_u, grasp_v = grasp_points_uv[min(best_idx, len(grasp_points_uv) - 1)]
            else:
                self.get_logger().warn("Falha ao carregar Ultralytics SAM2.")
                best_mask = None

            t3 = time.time()
            selection_ms = (t3 - t2) * 1000
            
            # Step 3: SAM2VideoPredictor — init por ponto (mascara full-res falha no predictor de video Ultralytics)
            img_np = np.array(img_pil.convert("RGB"))
            if best_mask is not None:
                init_pt = [[grasp_u, grasp_v]]
            else:
                init_pt = [[(x1 + x2) // 2, (y1 + y2) // 2]]
            with self._gpu_lock:
                init_results = self.video_predictor(
                    img_np,
                    points=init_pt,
                    labels=[1],
                )
            best_tracked_mask, score = _best_mask_from_results(init_results)
            if best_tracked_mask is None:
                self.get_logger().warn("Tracker inicializou sem máscara válida.")
                self.detection_running = False
                return
            mask_np = best_tracked_mask.astype(np.uint8)
            init_mask_area = int(mask_np.sum())
            self._adaptive_tracking_min_mask_area = max(300, int(init_mask_area * 0.45))
            
            self.last_tracking_score = score
            self.tracking_active = True
            self.tracking_frame_count = 1
            
            # Save grasp point offset for continuous TF tracking
            M = cv2.moments(mask_np)
            if M["m00"] != 0:
                mask_u = int(M["m10"] / M["m00"])
                mask_v = int(M["m01"] / M["m00"])
                self.grasp_offset = (grasp_u - mask_u, grasp_v - mask_v)
            else:
                self.grasp_offset = (0, 0)
            
            # Compute centroid and update TF
            centroid_uv = self._mask_to_tf(mask_np, self.latest_depth.copy(), score,
                                            depth_header=snapshot_header,
                                            cached_transform=cached_transform)
            self._publish_segmented_depth(mask_np, self.latest_depth.copy())
            self._publish_segmentation_mask(mask_np)
            # Visualization
            if self.visualize:
                self._update_display(self.latest_rgb, mask_np, score, centroid_uv)
            
            total_ms = (time.time() - t_start) * 1000
            self.get_logger().info(
                f"✅ Initial detection complete (score={score:.3f})\n"
                f"    [Timing] Qwen: {qwen_ms:.0f}ms | Selection: {selection_ms:.0f}ms | Total: {total_ms:.0f}ms"
            )
        
        except Exception as e:
            self.get_logger().error(f"Initial detection error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.detection_running = False

    def _secondary_cam_order(self):
        """Ordem fixa para lock GPU: prioridade explícita, senão ordem de secondary_camera_names."""
        names = list(self._secondary_camera_names)
        prio = self._secondary_tracking_priority
        if not prio:
            return names
        ordered = []
        for p in prio:
            if p in names and p not in ordered:
                ordered.append(p)
        for n in names:
            if n not in ordered:
                ordered.append(n)
        return ordered

    def _release_secondary_predictor(self, cam_name: str):
        st = self._secondary_cam_state.get(cam_name)
        if st is None:
            return
        pred = st.get('video_predictor')
        if pred is not None:
            try:
                pred.inference_state = {}
                if hasattr(pred, '_ros_frame_idx'):
                    pred._ros_frame_idx = 0
            except Exception:
                pass
            st['video_predictor'] = None
        st['tracking_initialized'] = False
        st['pending_secondary_init'] = False
        st['init_uv'] = None
        st['last_mask'] = None
        st['last_score'] = 0.0
        st['sec_frame_count'] = 0
        st['frame_pending'] = False

    def _maybe_resume_hand_after_secondary_stop(self):
        """Opcional: quando uma frontal deixa de seguir, retomar deteção na mão."""
        r = self.resume_hand_after_secondary_stop
        if r == 'none':
            return
        if self.detection_running:
            return
        if self.tracking_active:
            return
        if r in ('qwen', 'point'):
            self._run_initial_detection()

    def _run_hand_tracking_step(self, now: float):
        """Um passo de tracking SAM2 na mão (chamado com intervalo adaptativo)."""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if self.camera_intrinsics is None:
            return
        if not self.new_frame_available:
            return
        if (now - self._last_track_time) < self.active_tracking_interval:
            return

        self.new_frame_available = False
        self._last_track_time = now
        t_start = now

        try:
            img_pil = self.latest_rgb
            depth_map = self.latest_depth.copy()
            # Atomic snapshot: freeze depth header at same instant as image data.
            snapshot_header = getattr(self, 'latest_depth_header', None)

            # Pre-lookup TF while sensor stamp is still fresh (~0.2s old).
            # After SAM2 inference (1-3s), the stamp would be stale → extrapolation.
            cached_transform = None
            try:
                source_frame = getattr(self, 'camera_frame_id', None)
                if not source_frame:
                    source_frame = snapshot_header.frame_id if snapshot_header is not None else 'hand_cam'
                cached_transform = self._lookup_transform_with_sensor_stamp(
                    self.target_parent_frame, source_frame, snapshot_header
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Pre-lookup TF falhou (será tentado novamente em _mask_to_tf): {e}",
                    throttle_duration_sec=5.0,
                )

            self.tracking_frame_count += 1
            with self._gpu_lock:
                tracked_results = self.video_predictor(
                    np.array(img_pil.convert("RGB")),
                )
            best_tracked_mask, score = _best_mask_from_results(tracked_results)
            if best_tracked_mask is None:
                score = 0.0
                mask_np = np.zeros_like(depth_map, dtype=np.uint8)
            else:
                mask_np = best_tracked_mask.astype(np.uint8)

            if self.gpu_yield:
                torch.cuda.synchronize()
                torch.cuda.empty_cache()

            self.last_tracking_score = score
            tracking_ms = (time.time() - t_start) * 1000

            mask_area = int(mask_np.sum())
            effective_min_area = int(
                self._adaptive_tracking_min_mask_area
                if self._adaptive_tracking_min_mask_area is not None
                else self.tracking_min_mask_area
            )
            is_lost_now = (mask_area < effective_min_area)
            if is_lost_now:
                self._tracking_lost_streak += 1
            else:
                self._tracking_lost_streak = 0

            if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                now_sec = time.time()
                if (now_sec - self._last_redetect_time) >= self.tracking_redetect_cooldown_sec:
                    self._last_redetect_time = now_sec
                    self.get_logger().warn(
                        f"⚠️ Tracking lost (score={score:.3f}, area={mask_area}, "
                        f"streak={self._tracking_lost_streak}). Re-detecting..."
                    )
                    self._reset_tracking()
                    self._run_initial_detection()
                    return

            centroid_uv = self._mask_to_tf(mask_np, depth_map, score,
                                            depth_header=snapshot_header,
                                            cached_transform=cached_transform)

            self._publish_segmented_depth(mask_np, depth_map)
            self._publish_segmentation_mask(mask_np)

            if self.visualize:
                self._update_display(self.latest_rgb, mask_np, score, centroid_uv)

            if self.tracking_frame_count % 5 == 0:
                self.get_logger().info(
                    f"📍 Tracking frame {self.tracking_frame_count}: "
                    f"score={score:.3f} | {tracking_ms:.0f}ms | ACTIVE"
                )

            if self.tracking_frame_count >= self.MAX_TRACKING_FRAMES:
                self.get_logger().info(
                    f"🔄 Resetting tracking after {self.MAX_TRACKING_FRAMES} frames "
                    f"(safety limit)..."
                )
                self._reset_tracking()
                self._run_initial_detection()

        except Exception as e:
            self.get_logger().error(f"Tracking error: {e}")
            import traceback
            traceback.print_exc()
            self._reset_tracking()

    def _run_secondary_camera_tracking(self, now: float, cam_name: str):
        if not self.enable_secondary_video_tracking or not self.enable_secondary_fov_trigger:
            return
        st = self._secondary_cam_state.get(cam_name)
        if st is None:
            return
        if not st.get('trigger_active'):
            return
        if st.get('latest_rgb_bgr') is None:
            return

        need_init = bool(st.get('pending_secondary_init') and st.get('init_uv') is not None)
        if not need_init and not st.get('frame_pending'):
            return
        if not need_init and (now - st['last_sec_track_time']) < self.secondary_tracking_interval:
            if self.visualize and self.show_secondary_debug_windows:
                self._update_secondary_display(
                    cam_name,
                    st,
                    in_view=True,
                    uv=st.get('last_projected_uv'),
                    z_cam=None,
                    odom_ok=True,
                    stale=False,
                    mask_overlay=st.get('last_mask'),
                )
            return

        img_bgr = st['latest_rgb_bgr']
        h, w = img_bgr.shape[:2]
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        try:
            if need_init:
                if st['video_predictor'] is None:
                    st['video_predictor'] = create_video_predictor()
                    if st['video_predictor'] is None:
                        self.get_logger().error(
                            f"[FOV {cam_name}] SAM2 secundário indisponível (create_video_predictor)."
                        )
                        if self.visualize and self.show_secondary_debug_windows:
                            self._update_secondary_display(
                                cam_name,
                                st,
                                in_view=True,
                                uv=st.get('last_projected_uv'),
                                z_cam=None,
                                odom_ok=True,
                                stale=False,
                                mask_overlay=st.get('last_mask'),
                            )
                        return
                iu, iv = st['init_uv']
                u = int(round(iu))
                v = int(round(iv))
                u = max(0, min(w - 1, u))
                v = max(0, min(h - 1, v))
                # #region agent log
                try:
                    _append_debug_log(
                        "H4",
                        "detect_qwen.py:_run_secondary_camera_tracking:need_init",
                        "secondary tracker initializing from trigger uv",
                        {
                            "cam_name": str(cam_name),
                            "init_uv_raw": [float(iu), float(iv)],
                            "init_uv_clamped": [int(u), int(v)],
                            "image_wh": [int(w), int(h)],
                            "trigger_active": bool(st.get("trigger_active")),
                            "pending_secondary_init": bool(st.get("pending_secondary_init")),
                        },
                    )
                except Exception:
                    pass
                # #endregion
                with self._gpu_lock:
                    res = st['video_predictor'](
                        img_rgb,
                        points=[[u, v]],
                        labels=[1],
                    )
                mask_np, score = _best_mask_from_results(res)
                if mask_np is None:
                    mask_np = np.zeros((h, w), dtype=np.uint8)
                    score = 0.0
                else:
                    mask_np = mask_np.astype(np.uint8)
                st['tracking_initialized'] = True
                st['pending_secondary_init'] = False
                st['frame_pending'] = False
                st['last_sec_track_time'] = now
                st['sec_frame_count'] = 1
                st['last_mask'] = mask_np
                st['last_score'] = score
            elif st.get('tracking_initialized') and st.get('video_predictor') is not None:
                with self._gpu_lock:
                    res = st['video_predictor'](img_rgb)
                mask_np, score = _best_mask_from_results(res)
                if mask_np is None:
                    h2, w2 = img_bgr.shape[:2]
                    mask_np = np.zeros((h2, w2), dtype=np.uint8)
                    score = 0.0
                else:
                    mask_np = mask_np.astype(np.uint8)
                st['frame_pending'] = False
                st['last_sec_track_time'] = now
                st['sec_frame_count'] = int(st.get('sec_frame_count', 0)) + 1
                st['last_mask'] = mask_np
                st['last_score'] = score

            if self.gpu_yield:
                torch.cuda.synchronize()

            if self.visualize and self.show_secondary_debug_windows:
                self._update_secondary_display(
                    cam_name,
                    st,
                    in_view=True,
                    uv=st.get('last_projected_uv'),
                    z_cam=None,
                    odom_ok=True,
                    stale=False,
                    mask_overlay=st.get('last_mask'),
                )
        except Exception as e:
            self.get_logger().error(f"[FOV {cam_name}] tracking secundário: {e}")
            import traceback
            traceback.print_exc()

    def _tracking_timer_cb(self):
        """Mão + frontais no mesmo tick; inferência SAM2 serializada com _gpu_lock."""
        if self.detection_running:
            return

        now = time.time()
        if not self.initial_detection_done and self._last_synced_pair_wall is None:
            if (now - self._node_start_wall) > 2.0 and (now - self._last_no_sync_warn_wall) > 2.0:
                self._last_no_sync_warn_wall = now
                stamp_delta = None
                if self._last_rgb_stamp_sec is not None and self._last_depth_stamp_sec is not None:
                    stamp_delta = float(abs(self._last_rgb_stamp_sec - self._last_depth_stamp_sec))
                self.get_logger().warn(
                    "Aguardando par RGB/depth sincronizado (request ainda não inicia). "
                    f"slop={self.sync_slop_sec:.3f}s queue={self.sync_queue_size} "
                    f"rgb_taps={self._rgb_tap_count} depth_taps={self._depth_tap_count} "
                    f"last_dt={stamp_delta if stamp_delta is not None else 'n/a'}"
                )
        if self.tracking_active and not self.new_frame_available:
            if (now - self._last_stale_frame_warn_wall) > 2.0:
                self._last_stale_frame_warn_wall = now
                age = None
                if self._last_synced_pair_wall is not None:
                    age = float(now - self._last_synced_pair_wall)

        if not self.tracking_active:
            if (
                self.new_frame_available
                and self.latest_rgb is not None
                and self.latest_depth is not None
                and self.camera_intrinsics is not None
            ):
                self._run_initial_detection()
        else:
            self._run_hand_tracking_step(now)

        if self.enable_secondary_video_tracking and self.enable_secondary_fov_trigger:
            for cam_name in self._secondary_cam_order():
                self._run_secondary_camera_tracking(now, cam_name)

    def _update_display(self, img_pil, mask_np, score, centroid_uv=None):
        """Update OpenCV debug window with tracking overlay."""
        try:
            # Convert PIL → BGR for OpenCV
            img_bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
            
            # Apply mask overlay
            if mask_np is not None:
                # Green overlay
                overlay = np.zeros_like(img_bgr)
                overlay[mask_np > 0] = [0, 255, 0]
                img_bgr = cv2.addWeighted(img_bgr, 1.0, overlay, 0.5, 0)
                
                # Draw contour
                contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img_bgr, contours, -1, (0, 255, 0), 2)
            
            # Draw centroid and text
            if centroid_uv:
                u, v = int(centroid_uv[0]), int(centroid_uv[1])
                cv2.circle(img_bgr, (u, v), 5, (0, 0, 255), -1)
                text = f"Score: {score:.3f} | X: {u} Y: {v}"
            else:
                text = f"Score: {score:.3f}"
            
            cv2.putText(img_bgr, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show window
            cv2.imshow(self.tracking_window_name, img_bgr)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {e}")

    def _publish_segmentation_mask(self, mask_np):
        """Publish binary segmentation mask (mono8) for downstream nodes (e.g. Open3D PCD)."""
        if not self.publish_segmentation_mask:
            return
        m = ((mask_np > 0).astype(np.uint8) * 255)
        try:
            msg = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            if hasattr(self, 'latest_depth_header') and self.latest_depth_header is not None:
                msg.header = self.latest_depth_header
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.camera_frame_id or 'hand_cam'
            self.mask_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing segmentation mask: {e}")

    def _publish_segmented_depth(self, mask_np, depth_map):
        """Publish the segmented depth image."""
        if self.depth_seg_pub.get_subscription_count() > 0:
            seg_depth = np.full_like(depth_map, np.nan, dtype=depth_map.dtype)
            seg_depth[mask_np > 0] = depth_map[mask_np > 0]
            
            try:
                msg = self.bridge.cv2_to_imgmsg(seg_depth, encoding='32FC1')
                if hasattr(self, 'latest_depth_header'):
                    msg.header = self.latest_depth_header
                else:
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'hand_cam'
                self.depth_seg_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing segmented depth: {e}")

    def _reset_tracking(self):
        """Reset the tracking state for a fresh start."""
        self.tracking_active = False
        self.tracking_frame_count = 0
        self._tracking_lost_streak = 0
        self.new_frame_available = False
        self._last_track_time = 0.0
        if self.video_predictor is not None:
            self.video_predictor.inference_state = {}
            if hasattr(self.video_predictor, "_ros_frame_idx"):
                self.video_predictor._ros_frame_idx = 0
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        self._consistency_ref_ema = None

    def _get_valid_depth(self, depth_map, u, v, radius=3):
        """Get valid depth (in meters) at pixel (u, v) with a search window.
        
        Auto-detects mm vs m based on value range.
        Returns (Z_meters, success).
        """
        h, w = depth_map.shape[:2]
        v_c = min(max(0, v), h - 1)
        u_c = min(max(0, u), w - 1)
        
        v_min, v_max = max(0, v_c - radius), min(h, v_c + radius + 1)
        u_min, u_max = max(0, u_c - radius), min(w, u_c + radius + 1)
        patch = depth_map[v_min:v_max, u_min:u_max]
        
        finite = patch[np.isfinite(patch) & (patch > 0)]
        if len(finite) == 0:
            return 0.0, False
        
        # Auto-detect mm vs m: if median > 20, assume millimeters
        med = float(np.median(finite))
        if med > 20.0:
            finite = finite / 1000.0  # mm -> m
            med = med / 1000.0
        
        # Filter plausible range in meters
        valid = finite[(finite > 0.05) & (finite < 10.0)]
        if len(valid) == 0:
            return 0.0, False
        
        return float(np.median(valid)), True

    def _mask_to_tf(self, mask_np, depth_map, score, depth_header=None, cached_transform=None):
        """Compute 3D centroid from mask and publish TF. Returns (u, v).

        Args:
            depth_header: snapshot of depth header captured at the same time as the
                          image data. Used as fallback for TF lookup if cached_transform
                          is not provided.
            cached_transform: TransformStamped pre-looked-up before inference (while
                              sensor stamp was still fresh). When provided, skips the
                              live TF lookup entirely — avoids stale-stamp extrapolation.
        """
        # Find centroid of the mask
        M = cv2.moments(mask_np)
        if M["m00"] == 0:
            # #region agent log
            try:
                _append_debug_log(
                    "H5",
                    "detect_qwen.py:_mask_to_tf:zero_area_mask",
                    "mask_to_tf received zero-area mask",
                    {
                        "tracking_frame_count": int(self.tracking_frame_count),
                        "score": float(score),
                        "mask_sum": int(mask_np.sum()),
                        "tracking_active": bool(self.tracking_active),
                    },
                )
            except Exception:
                pass
            # #endregion
            self.get_logger().warn("⚠️ _mask_to_tf: mask has zero area (m00=0)")
            return None
        
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        
        if hasattr(self, 'grasp_offset'):
            u += self.grasp_offset[0]
            v += self.grasp_offset[1]
        
        # Clamp to bounds
        h, w = depth_map.shape[:2]
        v = min(max(0, v), h - 1)
        u = min(max(0, u), w - 1)
        
        # Strategy: try centroid first, then grasp points, then expand window
        Z, ok = self._get_valid_depth(depth_map, u, v, radius=3)
        
        if not ok and hasattr(self, 'grasp_points_uv_stored'):
            # Fallback 1: try each grasp point
            for pt in self.grasp_points_uv_stored:
                Z, ok = self._get_valid_depth(depth_map, pt[0], pt[1], radius=5)
                if ok:
                    u, v = pt[0], pt[1]
                    self.get_logger().info(
                        f"🔄 Depth fallback: using grasp point ({u},{v}) Z={Z:.3f}m"
                    )
                    break
        
        if not ok:
            # Fallback 2: search wider window around mask centroid
            for r in [10, 20, 40]:
                Z, ok = self._get_valid_depth(depth_map, u, v, radius=r)
                if ok:
                    self.get_logger().info(
                        f"🔄 Depth fallback: expanded window r={r} -> Z={Z:.3f}m"
                    )
                    break
        
        if not ok:
            # Fallback 3: scan entire mask region for any valid depth
            ys, xs = np.where(mask_np > 0)
            if len(ys) > 0:
                mask_depths = depth_map[ys, xs].astype(np.float64)
                finite = mask_depths[np.isfinite(mask_depths) & (mask_depths > 0)]
                if len(finite) > 0:
                    med = float(np.median(finite))
                    if med > 20.0:
                        med = med / 1000.0
                    if 0.05 < med < 10.0:
                        Z = med
                        ok = True
                        self.get_logger().info(
                            f"🔄 Depth fallback: full mask scan -> Z={Z:.3f}m "
                            f"({len(finite)} valid pixels)"
                        )
        
        if not ok:
            self.get_logger().warn(
                f"⚠️ _mask_to_tf: No valid depth found anywhere! "
                f"centroid=({u},{v}), depth dtype={depth_map.dtype}"
            )
            return (u, v)
        
        # Back-project
        fx, fy, cx, cy = self.camera_intrinsics
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        
        # Transform to robot base (see robot_base_frame parameter)
        try:
            # Use cached transform if available (pre-looked-up before inference)
            if cached_transform is not None:
                t = cached_transform
                source_frame = t.child_frame_id  # for error logging only
            else:
                sensor_h = depth_header if depth_header is not None else getattr(self, 'latest_depth_header', None)
                source_frame = sensor_h.frame_id if sensor_h is not None else 'hand_cam'
                self.get_logger().info(
                    f"🔎 _mask_to_tf: centroid=({u},{v}) Z={Z:.3f}m -> cam3D=({X:.3f},{Y:.3f},{Z:.3f}) "
                    f"source_frame='{source_frame}'",
                    once=True
                )
                t = self._lookup_transform_with_sensor_stamp(
                    self.target_parent_frame,
                    source_frame,
                    sensor_h,
                )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            rx, ry, rz, rw = (t.transform.rotation.x, t.transform.rotation.y, 
                              t.transform.rotation.z, t.transform.rotation.w)
            
            # Rotate point
            px, py, pz = self._rotate_point_by_quaternion(X, Y, Z, rx, ry, rz, rw)
            
            target_x = px + tx
            target_y = py + ty
            target_z = pz + tz
            
            # Publish TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.target_parent_frame
            tf_msg.child_frame_id = self.target_frame_name
            tf_msg.transform.translation.x = target_x
            tf_msg.transform.translation.y = target_y
            tf_msg.transform.translation.z = target_z
            tf_msg.transform.rotation.w = 1.0
            
            self.target_transform = tf_msg
            self._target_tf_wall_time = time.time()
            self._target_pose_measure_wall_time = self._target_tf_wall_time
            self._target_pose_seq += 1
            self.tf_broadcaster.sendTransform(tf_msg)
            
            # Log periodically (every 10th frame)
            if self.tracking_frame_count % 10 == 0:
                self.get_logger().info(
                    f"📢 Publishing TF: {self.robot_base_frame} -> {self.target_frame_name} at "
                    f"({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
                )
            
        except Exception as e:
            # Diagnóstico detalhado já foi emitido em _lookup_transform_with_sensor_stamp (throttle 5s).
            self.get_logger().error(
                f"❌ _mask_to_tf: falha no lookup TF (source_frame='{source_frame}'): {e}"
            )
            
        return (u, v)

    def _rotate_point_by_quaternion(self, px, py, pz, qx, qy, qz, qw):
        """Rotate a 3D point by a quaternion."""
        t0 = 2.0 * (qy * pz - qz * py)
        t1 = 2.0 * (qz * px - qx * pz)
        t2 = 2.0 * (qx * py - qy * px)
        
        rx = px + qw * t0 + (qy * t2 - qz * t1)
        ry = py + qw * t1 + (qz * t0 - qx * t2)
        rz = pz + qw * t2 + (qx * t1 - qy * t0)
        
        return rx, ry, rz




    def _tf_publish_cb(self):
        """Publish stored TF at 10 Hz."""
        if self.target_transform is not None:
            self.target_transform.header.stamp = self.get_clock().now().to_msg()
            # Enquanto há tracking ativo, republicar o target a 10 Hz deve manter o alvo "fresco"
            # para o gate FOV (evita stale_target só porque _mask_to_tf não corre há >max_target_pose_age_sec).
            if self.tracking_active:
                self._target_tf_wall_time = time.time()
            self.tf_broadcaster.sendTransform(self.target_transform)
        self._update_secondary_fov_triggers()

    def _secondary_camera_info_cb(self, msg: CameraInfo, cam_name: str):
        st = self._secondary_cam_state.get(cam_name)
        if st is None:
            return
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        st['info_msg'] = msg
        st['intrinsics'] = (fx, fy, cx, cy)
        st['frame_id'] = msg.header.frame_id
        st['width'] = int(msg.width)
        st['height'] = int(msg.height)
        if self.secondary_fov_ignore_distortion:
            st['d_coeffs'] = None
        else:
            st['d_coeffs'] = _camera_info_distortion_coeffs(msg)
        self.get_logger().info(
            f"[FOV {cam_name}] camera_info {st['width']}x{st['height']} "
            f"frame_id={st['frame_id']}",
            once=True,
        )

    def _secondary_rgb_cb(self, msg: RosImage, cam_name: str):
        st = self._secondary_cam_state.get(cam_name)
        if st is None:
            return
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            st['latest_rgb_bgr'] = cv_bgr
            st['frame_pending'] = True
        except Exception:
            # Silencioso para não poluir o loop principal.
            return
        # Propaga máscara + UV no frame mais recente (não só no timer TF / tracking).
        if self.visualize and self.show_secondary_debug_windows:
            self._update_secondary_display(
                cam_name,
                st,
                in_view=bool(st.get('trigger_active')),
                uv=st.get('last_projected_uv'),
                z_cam=None,
                odom_ok=True,
                stale=False,
                mask_overlay=st.get('last_mask'),
            )

    def _update_secondary_display(
        self,
        cam_name: str,
        st: dict,
        in_view: bool,
        uv=None,
        z_cam=None,
        odom_ok=True,
        stale=False,
        mask_overlay=None,
    ):
        if not self.visualize or not self.show_secondary_debug_windows:
            return
        img = st.get('latest_rgb_bgr', None)
        if img is None:
            return
        try:
            out = img.copy()
            h, w = out.shape[:2]
            if mask_overlay is not None:
                mo = mask_overlay
                if mo.shape[:2] != (h, w):
                    mo = cv2.resize(mo, (w, h), interpolation=cv2.INTER_NEAREST)
                mo_u8 = (mo > 0).astype(np.uint8) * 255
                overlay = np.zeros_like(out)
                overlay[mo_u8 > 0] = [0, 255, 0]
                out = cv2.addWeighted(out, 1.0, overlay, 0.5, 0)
                contours, _ = cv2.findContours(mo_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(out, contours, -1, (0, 255, 0), 2)
            sc = st.get('last_score')
            if sc is not None and st.get('tracking_initialized'):
                cv2.putText(
                    out,
                    f"seg={float(sc):.3f}",
                    (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
            if uv is not None:
                u, v = int(round(uv[0])), int(round(uv[1]))
                u = max(0, min(w - 1, u))
                v = max(0, min(h - 1, v))
                cv2.circle(out, (u, v), 6, (0, 0, 255), -1)
                cv2.putText(out, f"u={u} v={v}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 220, 0), 2)
            status = "ON" if st.get('trigger_active', False) else "OFF"
            txt = f"{cam_name} trig={status} in_view={int(bool(in_view))} odom_ok={int(bool(odom_ok))} stale={int(bool(stale))}"
            cv2.putText(out, txt, (10, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            if z_cam is not None:
                cv2.putText(out, f"z={z_cam:.3f}m", (10, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            win = f"FOV Debug - {cam_name}"
            cv2.imshow(win, out)
            cv2.waitKey(1)
        except Exception:
            return

    def _transform_point_between_frames(self, out_frame: str, in_frame: str, stamp, p_in):
        """
        p em in_frame -> mesmo ponto expresso em out_frame.
        lookup(out_frame, in_frame): p_out = R * p_in + t.
        Evita usar target_object no buffer (TransformBroadcaster local por vezes não ecoa).
        """
        t = self._lookup_transform_robust(
            out_frame,
            in_frame,
            stamp,
            Duration(seconds=self.secondary_fov_tf_lookup_timeout_sec),
            f"{out_frame}<-{in_frame}",
        )
        if t is None:
            return None
        tr = t.transform.translation
        r = t.transform.rotation
        px, py, pz = float(p_in[0]), float(p_in[1]), float(p_in[2])
        rx, ry, rz = self._rotate_point_by_quaternion(px, py, pz, r.x, r.y, r.z, r.w)
        p_out = np.array([rx + tr.x, ry + tr.y, rz + tr.z], dtype=np.float64)
        return p_out

    def _lookup_transform_robust(
        self,
        target_frame: str,
        source_frame: str,
        stamp,
        timeout: Duration,
        context_tag: str,
    ):
        """
        Tenta lookup no stamp pedido; em falha por extrapolation/clock mismatch,
        faz fallback para último TF disponível (time=0).
        """
        if self.use_wall_time_tf_lookups:
            wall_to = Duration(seconds=self.secondary_fov_tf_lookup_timeout_sec)
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=wall_to,
                )
            except Exception as e_now:
                try:
                    return self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(seconds=0, nanoseconds=0),
                        timeout=wall_to,
                    )
                except Exception as e_t0:
                    self.get_logger().warn(
                        f"[FOV] TF lookup falhou ({context_tag}) no modo wall: "
                        f"now='{e_now}' | t0='{e_t0}'",
                        throttle_duration_sec=5.0,
                    )
                    return None

        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=timeout,
            )
        except Exception as e_first:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(seconds=0, nanoseconds=0),
                    timeout=timeout,
                )
            except Exception as e_second:
                try:
                    return self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=timeout,
                    )
                except Exception as e_third:
                    self.get_logger().warn(
                        f"[FOV] TF lookup falhou ({context_tag}): "
                        f"stamp='{e_first}' | t0='{e_second}' | now='{e_third}'",
                        throttle_duration_sec=5.0,
                    )
                    return None

    def _fov_target_pose_freshness_wall_time(self):
        """
        Instante (wall clock) em que o gate FOV considera o alvo "atualizado".

        _mask_to_tf só corre quando há par RGB/depth novo; pode haver >max_target_pose_age_sec
        sem nova medição mesmo com tracking ativo e TF republicado a 10 Hz (_target_tf_wall_time).
        Usar max(medição, último republish) evita loop STOP/START stale_target nesse caso.
        """
        m = self._target_pose_measure_wall_time
        t = self._target_tf_wall_time
        if self.tracking_active and t is not None:
            if m is not None:
                return max(m, t)
            return t
        return m

    def _update_secondary_fov_triggers(self):
        """FOV geométrico + consistência odom; atualiza triggers e UV projetado para tracking secundário."""
        if not self.enable_secondary_fov_trigger or not self._secondary_camera_names:
            return

        now_wall = time.time()
        measure_time = self._fov_target_pose_freshness_wall_time()
        stale = (
            self.target_transform is None
            or measure_time is None
            or (now_wall - measure_time) > self.max_target_pose_age_sec
        )

        stamp = rclpy.time.Time()
        if self.target_transform is not None:
            try:
                stamp = rclpy.time.Time.from_msg(self.target_transform.header.stamp)
            except Exception:
                pass

        odom_ok = True
        p_consistency = None
        if not stale:
            pb = np.array(
                [
                    self.target_transform.transform.translation.x,
                    self.target_transform.transform.translation.y,
                    self.target_transform.transform.translation.z,
                ],
                dtype=np.float64,
            )
            target_parent = str(self.target_transform.header.frame_id or self.target_parent_frame)
            if self.odom_consistency_frame == target_parent:
                p_consistency = pb.copy()
            else:
                p_consistency = self._transform_point_between_frames(
                    self.odom_consistency_frame, target_parent, stamp, pb
                )
                if p_consistency is None:
                    odom_ok = False

        if not stale and p_consistency is not None:
            if self._consistency_ref_ema is None:
                self._consistency_ref_ema = p_consistency.copy()
            dist = float(np.linalg.norm(p_consistency - self._consistency_ref_ema))
            # #region agent log
            try:
                _append_debug_log(
                    "H1",
                    "detect_qwen.py:_update_secondary_fov_triggers:consistency_check",
                    "odom consistency distance check",
                    {
                        "dist_m": dist,
                        "max_jump_m": float(self.max_consistency_position_jump_m),
                        "reject_streak": int(self._odom_reject_streak),
                        "target_parent_frame": str(self.target_parent_frame),
                        "odom_consistency_frame": str(self.odom_consistency_frame),
                        "p_consistency": [float(p_consistency[0]), float(p_consistency[1]), float(p_consistency[2])],
                        "ref_ema": [float(self._consistency_ref_ema[0]), float(self._consistency_ref_ema[1]), float(self._consistency_ref_ema[2])],
                    },
                )
            except Exception:
                pass
            # #endregion
            if dist > self.max_consistency_position_jump_m:
                self._odom_reject_streak += 1
                odom_ok = self._odom_reject_streak < self.odom_reject_confirm_ticks
                self.get_logger().warn(
                    f"[FOV] rejected_by_odom_error: salto={dist:.3f}m "
                    f"(limite {self.max_consistency_position_jump_m}m, "
                    f"ema_alpha={self.consistency_ema_alpha:.2f}, "
                    f"streak={self._odom_reject_streak}/{self.odom_reject_confirm_ticks})",
                    throttle_duration_sec=2.0,
                )
            else:
                self._odom_reject_streak = 0
            # Atualiza sempre a referência suavizada (EMA), mesmo em rejeição.
            a = self.consistency_ema_alpha
            self._consistency_ref_ema = (
                (1.0 - a) * self._consistency_ref_ema + a * p_consistency
            )

        for cam_name in self._secondary_camera_names:
            st = self._secondary_cam_state[cam_name]

            if stale:
                if st['trigger_active']:
                    self.get_logger().info(
                        f"[FOV {cam_name}] STOP trigger (stale_target) "
                        f"age>{self.max_target_pose_age_sec}s"
                    )
                self._release_secondary_predictor(cam_name)
                if st['trigger_active']:
                    self._maybe_resume_hand_after_secondary_stop()
                st['trigger_active'] = False
                st['prev_trigger'] = False
                st['in_view_streak'] = 0
                st['out_streak'] = 0
                self._update_secondary_display(
                    cam_name,
                    st,
                    in_view=False,
                    uv=None,
                    z_cam=None,
                    odom_ok=True,
                    stale=True,
                    mask_overlay=st.get('last_mask'),
                )
                continue

            if not odom_ok:
                # #region agent log
                try:
                    _append_debug_log(
                        "H2",
                        "detect_qwen.py:_update_secondary_fov_triggers:odom_rejected",
                        "secondary trigger blocked by odom rejection",
                        {
                            "cam_name": str(cam_name),
                            "trigger_active_before_block": bool(st.get("trigger_active")),
                            "reject_streak": int(self._odom_reject_streak),
                            "reject_confirm_ticks": int(self.odom_reject_confirm_ticks),
                            "in_view_streak": int(st.get("in_view_streak", 0)),
                            "out_streak": int(st.get("out_streak", 0)),
                        },
                    )
                except Exception:
                    pass
                # #endregion
                if st['trigger_active']:
                    self.get_logger().info(
                        f"[FOV {cam_name}] STOP trigger (rejected_by_odom_error)"
                    )
                self._release_secondary_predictor(cam_name)
                if st['trigger_active']:
                    self._maybe_resume_hand_after_secondary_stop()
                st['trigger_active'] = False
                st['prev_trigger'] = False
                st['in_view_streak'] = 0
                st['out_streak'] = 0
                self._update_secondary_display(
                    cam_name,
                    st,
                    in_view=False,
                    uv=None,
                    z_cam=None,
                    odom_ok=False,
                    stale=False,
                    mask_overlay=st.get('last_mask'),
                )
                continue

            if st['intrinsics'] is None or st['width'] <= 0 or st['height'] <= 0:
                continue

            cam_frame = st['frame_id']
            if not cam_frame:
                continue

            fx, fy, cx, cy = st['intrinsics']
            w, h = st['width'], st['height']
            d_use = None if self.secondary_fov_ignore_distortion else st['d_coeffs']

            prev_trig = st['prev_trigger']

            in_view_geom = False
            uv_dbg = None
            z_dbg = None
            pb = np.array(
                [
                    self.target_transform.transform.translation.x,
                    self.target_transform.transform.translation.y,
                    self.target_transform.transform.translation.z,
                ],
                dtype=np.float64,
            )
            target_parent = str(self.target_transform.header.frame_id or self.target_parent_frame)
            p_cam = self._transform_point_between_frames(cam_frame, target_parent, stamp, pb)
            if p_cam is not None:
                X, Y, Z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
                z_dbg = Z
                _u, _v, z_ok, in_image = _project_cam_ros_point_to_uv(
                    X, Y, Z, fx, fy, cx, cy, d_use, w, h
                )
                uv_dbg = (_u, _v)
                in_view_geom = bool(z_ok and in_image)

            if uv_dbg is not None:
                st['last_projected_uv'] = uv_dbg

            if in_view_geom:
                st['in_view_streak'] += 1
                st['out_streak'] = 0
            else:
                st['out_streak'] += 1
                st['in_view_streak'] = 0

            # Sincronização: não armar trigger em pose antiga (mesma base de frescura que stale).
            fresh_t = self._fov_target_pose_freshness_wall_time()
            pose_age_sec = (now_wall - fresh_t) if fresh_t is not None else 1e9
            if pose_age_sec > self.secondary_trigger_max_pose_age_sec:
                if in_view_geom:
                    in_view_geom = False
                    st['out_streak'] = 1
                    st['in_view_streak'] = 0
            if not st['trigger_active']:
                if st['in_view_streak'] >= self.fov_trigger_confirm_ticks:
                    st['trigger_active'] = True
                    # #region agent log
                    try:
                        _append_debug_log(
                            "H3",
                            "detect_qwen.py:_update_secondary_fov_triggers:start_trigger",
                            "secondary trigger turned ON",
                            {
                                "cam_name": str(cam_name),
                                "in_view_streak": int(st.get("in_view_streak", 0)),
                                "pose_age_sec": float(pose_age_sec),
                                "projected_uv": list(st.get("last_projected_uv")) if st.get("last_projected_uv") is not None else None,
                            },
                        )
                    except Exception:
                        pass
                    # #endregion
                    self.get_logger().info(
                        f"[FOV {cam_name}] START trigger (entered_fov) "
                        f"confirmações={self.fov_trigger_confirm_ticks}"
                    )
            else:
                if st['out_streak'] >= self.fov_release_confirm_ticks:
                    st['trigger_active'] = False
                    self.get_logger().info(
                        f"[FOV {cam_name}] STOP trigger (left_fov) "
                        f"confirmações={self.fov_release_confirm_ticks}"
                    )

            if st['trigger_active'] and not prev_trig and self.enable_secondary_video_tracking:
                luv = st.get('last_projected_uv')
                if luv is not None:
                    st['pending_secondary_init'] = True
                    st['init_uv'] = luv
                    # #region agent log
                    try:
                        _append_debug_log(
                            "H4",
                            "detect_qwen.py:_update_secondary_fov_triggers:pending_init_set",
                            "secondary pending init set from projected uv",
                            {
                                "cam_name": str(cam_name),
                                "init_uv": [int(luv[0]), int(luv[1])],
                                "prev_trigger": bool(prev_trig),
                            },
                        )
                    except Exception:
                        pass
                    # #endregion
            if prev_trig and not st['trigger_active']:
                self._release_secondary_predictor(cam_name)
                self._maybe_resume_hand_after_secondary_stop()
            st['prev_trigger'] = st['trigger_active']

            self._update_secondary_display(
                cam_name,
                st,
                in_view=in_view_geom,
                uv=uv_dbg,
                z_cam=z_dbg,
                odom_ok=True,
                stale=False,
                mask_overlay=st.get('last_mask'),
            )


def main(args=None):
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    node = None
    try:
        node = DetectQwenNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if 'executor' in locals():
            executor.shutdown()
        cv2.destroyAllWindows()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()


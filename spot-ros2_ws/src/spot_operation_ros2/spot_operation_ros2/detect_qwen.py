import base64
import json
import re
import sys
import os
import io
import time
from pathlib import Path

# Inject venv site-packages so SAM2/torch are found when running via ros2 run
_VENV_SITE = "/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/venv_valve_detection/lib/python3.10/site-packages"
if _VENV_SITE not in sys.path:
    sys.path.insert(0, _VENV_SITE)

# Also inject the source path for SAM2 (editable install) as .pth files aren't processed
_SAM2_ROOT = "/home/spot-teleop/spot-ros2_ws/segment-anything-2"
if _SAM2_ROOT not in sys.path:
    # Insert at 1 (after venv site-packages)
    sys.path.insert(1, _SAM2_ROOT)

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
    from sensor_msgs.msg import Image as RosImage, CameraInfo
    from geometry_msgs.msg import TransformStamped, PointStamped
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    import cv2
    import tf2_ros
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    import message_filters
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Aviso: Bibliotecas ROS 2 (rclpy, sensor_msgs, cv_bridge) não encontradas. Modo câmera indisponível.")


# Imports de segmentacao - Official SAM2
try:
    import torch
    from sam2.build_sam import build_sam2, build_sam2_video_predictor
    from sam2.sam2_image_predictor import SAM2ImagePredictor
    from sam2.sam2_video_predictor import SAM2VideoPredictor
    SAM2_AVAILABLE = True
except ImportError:
    SAM2_AVAILABLE = False
    print("Aviso: 'sam2' (official) nao encontrado. Segmentacao SAM desativada.")

from collections import OrderedDict

VLLM_URL = "http://localhost:8000"
# ===============================================

# Official SAM2 paths (absolute)
SAM2_CHECKPOINT = "/home/spot-teleop/spot-ros2_ws/checkpoints/sam2.1_hiera_base_plus.pt"
SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_b+.yaml"

# ImageNet normalization constants (same as SAM2 uses internally)
_IMG_MEAN = (0.485, 0.456, 0.406)
_IMG_STD = (0.229, 0.224, 0.225)

# Global SAM Model (ImagePredictor for CLI)
sam_model = None
# Global SAM Video Model (VideoPredictor for ROS node)
sam_video_model = None

def load_sam_model():
    """Load SAM2 ImagePredictor (used by CLI mode and segment_result)."""
    global sam_model
    if not SAM2_AVAILABLE:
        return None
    
    if sam_model is None:
        try:
            print(f"Carregando modelo SAM2 oficial ({SAM2_CHECKPOINT})...")
            sam2_model = build_sam2(SAM2_CONFIG, SAM2_CHECKPOINT, device="cuda")
            sam_model = SAM2ImagePredictor(sam2_model)
        except Exception as e:
            print(f"Erro ao carregar SAM2: {e}")
            import traceback
            traceback.print_exc()
            return None
    return sam_model


def load_sam_video_model():
    """Load SAM2 VideoPredictor (used by ROS node for streaming tracking)."""
    global sam_video_model
    if not SAM2_AVAILABLE:
        return None
    
    if sam_video_model is None:
        try:
            print(f"Carregando SAM2 VideoPredictor ({SAM2_CHECKPOINT})...")
            sam_video_model = build_sam2_video_predictor(
                SAM2_CONFIG, SAM2_CHECKPOINT, device="cuda"
            )
        except Exception as e:
            print(f"Erro ao carregar SAM2 VideoPredictor: {e}")
            import traceback
            traceback.print_exc()
            return None
    return sam_video_model


def init_streaming_state(predictor, video_height, video_width):
    """
    Manually create an inference_state for streaming (no video file).
    Mirrors what SAM2VideoPredictor.init_state() does internally.
    """
    compute_device = predictor.device
    inference_state = {
        "images": [],  # will grow as frames arrive
        "num_frames": 0,
        "offload_video_to_cpu": False,
        "offload_state_to_cpu": False,
        "video_height": video_height,
        "video_width": video_width,
        "device": compute_device,
        "storage_device": compute_device,
        "point_inputs_per_obj": {},
        "mask_inputs_per_obj": {},
        "cached_features": {},
        "constants": {},
        "obj_id_to_idx": OrderedDict(),
        "obj_idx_to_id": OrderedDict(),
        "obj_ids": [],
        "output_dict_per_obj": {},
        "temp_output_dict_per_obj": {},
        "frames_tracked_per_obj": {},
    }
    return inference_state


def append_frame_to_state(inference_state, predictor, pil_img):
    """
    Preprocess a PIL RGB image and append it to the streaming inference state.
    Returns the frame index of the newly added frame.
    """
    img_size = predictor.image_size  # model's internal resolution (e.g. 1024)
    img_np = np.array(pil_img.convert("RGB").resize((img_size, img_size))) / 255.0
    img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float()
    
    # Normalize with ImageNet mean/std
    mean = torch.tensor(_IMG_MEAN, dtype=torch.float32)[:, None, None]
    std = torch.tensor(_IMG_STD, dtype=torch.float32)[:, None, None]
    img_tensor = (img_tensor - mean) / std
    
    # Move to device
    device = inference_state["device"]
    img_tensor = img_tensor.to(device)
    
    frame_idx = inference_state["num_frames"]
    inference_state["images"].append(img_tensor)
    inference_state["num_frames"] = frame_idx + 1
    
    return frame_idx


def save_triposr_crop(img_pil, mask_np, output_path="/tmp/triposr_crop.png", foreground_ratio=0.85):
    """
    Cria um crop da imagem centrado no mask_np, usando o foreground_ratio especificado.
    """
    # Extract tight bounding box of the mask
    ys, xs = np.where(mask_np > 0)
    if len(ys) == 0 or len(xs) == 0:
        return None
        
    x_min, x_max = np.min(xs), np.max(xs)
    y_min, y_max = np.min(ys), np.max(ys)
    
    # Calculate object size
    obj_w = x_max - x_min
    obj_h = y_max - y_min
    
    # Calculate target image size to achieve the foreground_ratio
    max_obj_dim = max(obj_w, obj_h)
    target_size = int(max_obj_dim / foreground_ratio)
    
    # Calculate center of the object
    cx = (x_min + x_max) // 2
    cy = (y_min + y_max) // 2
    
    # Calculate crop coordinates based on target_size square
    half_size = target_size // 2
    crop_x1 = cx - half_size
    crop_y1 = cy - half_size
    crop_x2 = crop_x1 + target_size
    crop_y2 = crop_y1 + target_size
    
    # Pad handling
    orig_w, orig_h = img_pil.size
    
    src_x1 = max(0, crop_x1)
    src_y1 = max(0, crop_y1)
    src_x2 = min(orig_w, crop_x2)
    src_y2 = min(orig_h, crop_y2)
    
    if src_x1 >= src_x2 or src_y1 >= src_y2:
        return None
        
    dst_x1 = src_x1 - crop_x1
    dst_y1 = src_y1 - crop_y1
    
    # Crop the original image
    src_crop = img_pil.crop((src_x1, src_y1, src_x2, src_y2))
    
    mask_patch = mask_np[src_y1:src_y2, src_x1:src_x2]
    mask_crop = Image.fromarray((mask_patch * 255).astype(np.uint8)).convert("L")
    
    src_rgba = src_crop.convert("RGBA")
    src_rgba.putalpha(mask_crop)
    
    # We want a gray background for TripoSR (it expects RGB with ~0.5 background)
    img_padded_rgba = Image.new("RGBA", (target_size, target_size), (127, 127, 127, 0))
    img_padded_rgba.paste(src_rgba, (dst_x1, dst_y1), src_rgba) # paste with alpha
    
    # Remove alpha to just have gray bg
    final_rgb = Image.new("RGB", (target_size, target_size), (127, 127, 127))
    final_rgb.paste(img_padded_rgba, mask=img_padded_rgba.split()[3])
    
    final_rgb.save(output_path)
    print(f"Salvou crop para TripoSR em {output_path}")
    return output_path


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


def ask_qwen_selection(mosaic_path, object_name):
    """
    Perguntar ao Qwen qual a melhor mascara.
    """
    base64_img = encode_image_to_base64(mosaic_path)
    
    prompt = f"""Look at the 3 images labeled 1, 2, and 3.
They show different segmentation masks (green overlay) for a '{object_name}'.
Which mask represents the best mask segmentation?
Return ONLY the digit: 1, 2, or 3."""

    payload = {
        "model": "Qwen/Qwen3-VL-8B-Instruct", 
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_img}"}},
                    {"type": "text", "text": prompt}
                ]
            }
        ],
        "max_tokens": 10,
        "temperature": 0.01 
    }
    
    try:
        print("Perguntando ao Qwen qual a melhor mascara...")
        response = requests.post(f"{VLLM_URL}/v1/chat/completions", json=payload, timeout=30)
        response.raise_for_status()
        content = response.json()['choices'][0]['message']['content']
        print(f"Qwen escolheu: {content}")
        
        # Extract digit
        match = re.search(r'\b([1-3])\b', content)
        if match:
            return int(match.group(1)) - 1 # 0-indexed
    except Exception as e:
        print(f"Erro ao perguntar ao Qwen: {e}")
    
    return 0 # Fallback to first mask


def segment_result(image_input, boxes: list[dict], output_path: str):
    """
    Usa SAM para segmentar as caixas detectadas.
    Args:
        image_input: str (filepath) or PIL.Image
    Returns:
        dict with {'mask': np.ndarray, 'centroid_uv': (u, v), 'bbox': [x1,y1,x2,y2]} or None
    """
    if not SAM2_AVAILABLE:
        return None

    model = load_sam_model()
    if model is None:
        return None

    print("\nIniciando Segmentacao com SAM...")
    
    if isinstance(image_input, str) or isinstance(image_input, Path):
        img_pil = Image.open(image_input)
    elif isinstance(image_input, Image.Image):
        img_pil = image_input
    
    w, h = img_pil.size
    
    # Process ONLY the first box for now
    box_data = boxes[0] 
    label = box_data['label']
    b1000 = box_data['bbox_1000']
    
    x1 = int((b1000[0] / 1000.0) * w)
    y1 = int((b1000[1] / 1000.0) * h)
    x2 = int((b1000[2] / 1000.0) * w)
    y2 = int((b1000[3] / 1000.0) * h)
    
    # Clamp
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w-1, x2), min(h-1, y2)
    bbox = [x1, y1, x2, y2]
    
    print(f"Segmenting box: {bbox} for label: {label}")

    g1000 = box_data.get('grasp_1000')
    if g1000:
        cx = int((g1000[0] / 1000.0) * w)
        cy = int((g1000[1] / 1000.0) * h)
    else:
        # Use center point of bbox as prompt (NO box).
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

    try:
        # Official SAM2: set_image expects RGB uint8 numpy array (3 channels)
        img_rgb = img_pil.convert("RGB") if img_pil.mode != "RGB" else img_pil
        img_np = np.array(img_rgb)
        model.set_image(img_np)
        
        # Run Inference with Point-Only Prompt + multimask_output=True
        masks, scores, logits = model.predict(
            point_coords=np.array([[cx, cy]]),
            point_labels=np.array([1]),
            multimask_output=True,
        )
        
        print(f"SAM2 Raw Masks Shape: {masks.shape}")
        print(f"SAM2 Scores: {scores}")
        
        num_masks = masks.shape[0]
        selected_idx = 0
        
        if num_masks > 1:
            print(f"SAM2 retornou {num_masks} mascaras distintas. Selecionando a melhor...")
            # Create Mosaic
            mosaic_path = create_crop_mosaic(img_pil, masks, bbox, output_path="debug_mosaic.jpg")
            if mosaic_path:
                selected_idx = ask_qwen_selection(mosaic_path, label)
                print(f"Using Mask Index: {selected_idx} (Option {selected_idx+1})")
        
        # Get the selected mask
        final_mask = masks[selected_idx]
        if final_mask.ndim > 2: final_mask = final_mask.squeeze()
        
        # Compute mask centroid (u, v) in pixel coordinates
        ys, xs = np.where(final_mask > 0.5)
        if len(xs) > 0:
            centroid_u = int(np.mean(xs))
            centroid_v = int(np.mean(ys))
        else:
            centroid_u, centroid_v = cx, cy
        
        print(f"Mask centroid (u, v): ({centroid_u}, {centroid_v})")
        
        # Save overlay
        mask_pil = Image.fromarray((final_mask * 255).astype(np.uint8))
        if mask_pil.size != img_pil.size:
             mask_pil = mask_pil.resize(img_pil.size, Image.NEAREST)
        
        comp = img_pil.convert("RGBA").copy()
        green = Image.new("RGBA", comp.size, (0, 255, 0, 100))
        comp.paste(green, (0,0), mask_pil)
        
        comp.convert("RGB").save(output_path)
        print(f"Imagem segmentada salva em: {output_path}")
        
        return {
            'mask': final_mask,
            'centroid_uv': (centroid_u, centroid_v),
            'bbox': bbox,
        }

    except Exception as e:
        print(f"Erro na inferencia SAM2: {e}")
        import traceback
        traceback.print_exc()
        return None


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
    
    # 0. Try JSON Parsing first (since model seems to prefer it now)
    try:
        data = json.loads(clean_text)
        if isinstance(data, list):
            print("DEBUG: Detectado formato JSON.")
            if len(data) == 0:
                print("DEBUG: Lista vazia retornada (objeto não encontrado pelo Qwen).")
                return []
            for item in data:
                if 'bbox_2d' in item:
                    # JSON bbox_2d is typically [xmin, ymin, xmax, ymax] in PIXELS
                    b = item['bbox_2d']
                    label = item.get('label', 'object')
                    confidence = float(item.get('confidence', 1.0))
                    
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
                            gx, gy = pt[0], pt[1]
                            grasps_1000.append([int(gx), int(gy)])
                    
                    boxes.append({
                        'label': label,
                        'bbox_1000': [int(xmin), int(ymin), int(xmax), int(ymax)],
                        'grasps_1000': grasps_1000,
                        'confidence': confidence
                    })
            if boxes:
                return boxes
    except Exception as e:
        pass

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
        "model": "Qwen/Qwen3-VL-8B-Instruct", 
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


class SnapshotNode(Node):
    """One-shot image capture for CLI mode."""
    def __init__(self):
        super().__init__('camera_snapshot_node')
        self.image = None
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            RosImage,
            '/hand/rgb',
            self.listener_callback,
            10
        )
        print("Aguardando imagem em /hand/rgb ...")

    def listener_callback(self, msg):
        self.get_logger().info('Imagem recebida!')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.image = Image.fromarray(cv_image)
        except Exception as e:
            self.get_logger().error(f'Falha ao converter imagem: {e}')
        raise SystemExit 


def capture_image_from_ros():
    if not ROS_AVAILABLE:
        print("Erro: Bibliotecas ROS não disponíveis.")
        sys.exit(1)
        
    rclpy.init()
    node = SnapshotNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        img = node.image
        node.destroy_node()
        rclpy.shutdown()
        
    if img is None:
        print("Erro: Nenhuma imagem capturada.")
        sys.exit(1)
        
    return img


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
    
    # Sliding window: max frames to keep in inference state before reset
    # 9000 frames @ 5Hz = 30 minutes. 
    # We free old frame tensors to avoid OOM, so this limit is just a safety stop.
    MAX_TRACKING_FRAMES = 9000
    
    def __init__(self):
        super().__init__('detect_qwen_node')
        
        # Parameters
        self.declare_parameter('object_prompt', 'lever valve')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('target_frame', 'target_object')
        self.declare_parameter('rgb_topic', '/hand/rgb')
        self.declare_parameter('depth_topic', '/hand/depth')
        self.declare_parameter('camera_info_topic', '/hand/camera_info')
        self.declare_parameter('visualize', True)
        
        self.object_prompt = self.get_parameter('object_prompt').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_frame_name = self.get_parameter('target_frame').value
        self.visualize = self.get_parameter('visualize').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        self.get_logger().info(f"Object prompt: '{self.object_prompt}'")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"Target frame: {self.target_frame_name}")
        
        # State
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None   # (fx, fy, cx, cy)
        self.target_transform = None    # TransformStamped (odom → target)
        self.detection_running = False
        self.initial_detection_done = False
        
        # Tracking state
        self.declare_parameter('depth_segmented_topic', '/hand/depth_segmented')
        depth_segmented_topic = self.get_parameter('depth_segmented_topic').value
        self.depth_seg_pub = self.create_publisher(RosImage, depth_segmented_topic, 10)
        
        # TripoSR integration: publish crop image + call mesh generation service
        self.triposr_img_pub = self.create_publisher(RosImage, '/triposr/input_image', 1)
        self.mesh_client = self.create_client(Trigger, '/generate_mesh')
        
        self.video_predictor = None
        self.inference_state = None
        self.tracking_active = False
        self.current_frame_idx = -1
        self.new_frame_available = False
        self.tracking_frame_count = 0
        self.last_tracking_score = 0.0
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Camera info subscriber (one-shot, just need intrinsics)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_cb,
            1
        )
        
        # Synced RGB + Depth subscribers
        self.rgb_sub = message_filters.Subscriber(self, RosImage, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, RosImage, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=5, 
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self._synced_image_cb)
        
        # Tracking timer (5 Hz — processes new frames for tracking)
        self.tracking_timer = self.create_timer(0.2, self._tracking_timer_cb)
        
        # TF publish timer (10 Hz)
        self.tf_timer = self.create_timer(0.1, self._tf_publish_cb)
        
        # Pre-load SAM2 VideoPredictor
        self.get_logger().info("Pre-loading SAM2 VideoPredictor...")
        self.video_predictor = load_sam_video_model()
        if self.video_predictor is None:
            self.get_logger().error("Failed to load SAM2 VideoPredictor!")
        else:
            self.get_logger().info("SAM2 VideoPredictor loaded.")
        
        self.get_logger().info("DetectQwenNode ready. Waiting for camera data...")
    
    def _camera_info_cb(self, msg: CameraInfo):
        """Extract camera intrinsics once."""
        if self.camera_intrinsics is None:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.camera_intrinsics = (fx, fy, cx, cy)
            self.get_logger().info(
                f"Camera intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
            )
    
    def _synced_image_cb(self, rgb_msg: RosImage, depth_msg: RosImage):
        """Store latest synced RGB + Depth pair."""
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            self.latest_rgb = Image.fromarray(cv_rgb)
            
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            self.latest_depth_header = depth_msg.header
            self.new_frame_available = True
            
            # Trigger initial detection on first image
            if not self.initial_detection_done:
                self.initial_detection_done = True
                self.get_logger().info("First image received, triggering initial detection...")
                self._run_initial_detection()
                
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
    
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
        self.get_logger().info(f"🔍 Running initial detection for '{self.object_prompt}'...")
        
        try:
            img_pil = self.latest_rgb
            orig_w, orig_h = img_pil.size
            
            # Resize for Qwen
            max_dim = 1024
            scale_factor = max_dim / max(orig_w, orig_h)
            new_w = int(orig_w * scale_factor)
            new_h = int(orig_h * scale_factor)
            img_resized = img_pil.resize((new_w, new_h), Image.LANCZOS)
            
            # Step 1: Qwen detection
            t0 = time.time()
            response_text = detect_object(img_resized, self.object_prompt, VLLM_URL)
            t1 = time.time()
            qwen_ms = (t1 - t0) * 1000
            self.get_logger().info(f"Qwen response: {response_text[:60]}...")
            
            boxes = parse_qwen_response(response_text, image_size=(new_w, new_h))
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
            
            # Step 2: Generate Candidate Masks & Select Best
            t2 = time.time()
            self.get_logger().info("Generating candidate masks using SAM2 based on Qwen points...")
            
            sam_img_predictor = load_sam_model()
            if sam_img_predictor:
                sam_img_predictor.set_image(np.array(img_pil))
                
                all_masks = []
                # Combine points into a batch or query them one by one. One by one is easier to pick highest.
                for pt in grasp_points_uv:
                    masks, scores, _ = sam_img_predictor.predict(
                        point_coords=np.array([pt]),
                        point_labels=np.array([1]),
                        multimask_output=True,
                    )
                    # For each point, take the highest scoring mask according to SAM2
                    best_local_idx = np.argmax(scores)
                    all_masks.append(masks[best_local_idx])
                
                # Check ambiguity
                best_idx = 0
                if len(all_masks) > 1:
                    mosaic_path = create_crop_mosaic(img_pil, all_masks, bbox, "debug_mosaic.jpg")
                    if mosaic_path:
                        self.get_logger().info("Asking Qwen to select best mask from multiple generated choices...")
                        best_idx = ask_qwen_selection(mosaic_path, label)
                        self.get_logger().info(f"✅ Qwen selected mask {best_idx+1}")
                
                best_mask = all_masks[best_idx]
                # Update our primary tracking grasp point based on what Qwen picked
                grasp_u, grasp_v = grasp_points_uv[best_idx]
            else:
                self.get_logger().warn("ImagePredictor failed to load. Skipping selection step.")
                best_mask = None

            t3 = time.time()
            selection_ms = (t3 - t2) * 1000
            
            # Step 3: Initialize Video Tracker with selected mask
            self.inference_state = init_streaming_state(
                self.video_predictor, orig_h, orig_w
            )
            frame_idx = append_frame_to_state(
                self.inference_state, self.video_predictor, img_pil
            )
            self.current_frame_idx = frame_idx
            
            if best_mask is not None:
                # Use the selected mask to initialize tracking
                # SAM2 requires (H, W) mask tensor
                if best_mask.ndim > 2:
                     best_mask = best_mask.squeeze()
                
                mask_tensor = torch.from_numpy(best_mask).float().to(self.video_predictor.device)
                
                _, out_obj_ids, out_mask_logits = self.video_predictor.add_new_mask(
                    inference_state=self.inference_state,
                    frame_idx=frame_idx,
                    obj_id=1,
                    mask=mask_tensor
                )
            else:
                # Fallback to box prompt if no mask available
                _, out_obj_ids, out_mask_logits = self.video_predictor.add_new_points_or_box(
                    inference_state=self.inference_state,
                    frame_idx=frame_idx,
                    obj_id=1,
                    box=np.array(bbox, dtype=np.float32),
                )

            # Extract initial mask result
            mask_logit = out_mask_logits[0]
            mask_prob = torch.sigmoid(mask_logit)
            
            mask_pixels = mask_prob[0] > 0.5
            if mask_pixels.any():
                score = mask_prob[0][mask_pixels].mean().item()
            else:
                score = 0.0
                
            mask_np = mask_pixels.cpu().numpy().astype(np.uint8)
            
            # Publish crop for TripoSR service
            self._publish_triposr_crop(img_pil, mask_np, bbox)
            
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
            centroid_uv = self._mask_to_tf(mask_np, self.latest_depth.copy(), score)
            
            self._publish_segmented_depth(mask_np, self.latest_depth.copy())
            
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
    def _tracking_timer_cb(self):
        """Process new frames for SAM2 tracking at 5 Hz."""
        if self.detection_running:
            return
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if self.camera_intrinsics is None:
            return
            
        if not self.tracking_active:
            if self.new_frame_available:
                self._run_initial_detection()
            return
            
        if not self.new_frame_available:
            return
        
        self.new_frame_available = False
        t_start = time.time()
        
        try:
            img_pil = self.latest_rgb
            depth_map = self.latest_depth.copy()
            
            # Append new frame to inference state
            frame_idx = append_frame_to_state(
                self.inference_state, self.video_predictor, img_pil
            )
            self.current_frame_idx = frame_idx
            self.tracking_frame_count += 1
            
            # Propagate tracking to this frame
            for out_frame_idx, out_obj_ids, out_mask_logits in \
                    self.video_predictor.propagate_in_video(
                        self.inference_state,
                        start_frame_idx=frame_idx,
                        max_frame_num_to_track=1,
                    ):
                # out_mask_logits: (num_objects, 1, H, W)
                mask_logit = out_mask_logits[0]  # first (only) object
                mask_prob = torch.sigmoid(mask_logit)
                
                mask_pixels = mask_prob[0] > 0.5
                if mask_pixels.any():
                    score = mask_prob[0][mask_pixels].mean().item()
                else:
                    score = 0.0
                    
                mask_np = mask_pixels.cpu().numpy().astype(np.uint8)
            
            self.last_tracking_score = score
            tracking_ms = (time.time() - t_start) * 1000
            
            # Check if tracking is lost
            mask_area = mask_np.sum()
            if score < self.confidence_threshold or mask_area < 50:
                self.get_logger().warn(
                    f"⚠️ Tracking lost (score={score:.3f}, area={mask_area}). "
                    f"Re-detecting..."
                )
                self._reset_tracking()
                self._run_initial_detection()
                return
            
            # Compute centroid and update TF
            centroid_uv = self._mask_to_tf(mask_np, depth_map, score)
            
            self._publish_segmented_depth(mask_np, depth_map)
            
            # Visualization
            if self.visualize:
                self._update_display(self.latest_rgb, mask_np, score, centroid_uv)
            
            # Log periodically (every 5th frame to avoid spam)
            if self.tracking_frame_count % 5 == 0:
                self.get_logger().info(
                    f"📍 Tracking frame {self.tracking_frame_count}: "
                    f"score={score:.3f} | {tracking_ms:.0f}ms"
                )
            
            # Memory optimization: Free old frames (keep 0 and last 50)
            if frame_idx > 50:
                 # Ensure we don't free frame 0 (used for init conditioning)
                 old_idx = frame_idx - 50
                 if old_idx > 0 and old_idx < len(self.inference_state["images"]):
                     self.inference_state["images"][old_idx] = None
            
            # Safety reset if too many frames accumulated (30 minutes)
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
            # If tracking breaks, try to re-detect
            self._reset_tracking()

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
            cv2.imshow("SAM2 Live Tracking", img_bgr)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {e}")

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

    def _publish_triposr_crop(self, img_pil, mask_np, bbox, foreground_ratio=0.85):
        """
        Crop RGB image using the mask bounding box with margin,
        publish it, and call the TripoSR mesh generation service.
        """
        # Find exact mask bounds
        y_indices, x_indices = np.where(mask_np > 0)
        if len(y_indices) == 0 or len(x_indices) == 0:
            return
            
        y1, y2 = y_indices.min(), y_indices.max()
        x1, x2 = x_indices.min(), x_indices.max()
        
        # Center of the mask
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        
        # Size of the mask
        w, h = x2 - x1, y2 - y1
        size = max(w, h)
        
        # Add margin
        padded_size = int(size / foreground_ratio)
        half_size = padded_size // 2
        
        # Crop bounds with padding
        img_w, img_h = img_pil.size
        crop_x1 = max(0, cx - half_size)
        crop_y1 = max(0, cy - half_size)
        crop_x2 = min(img_w, cx + half_size)
        crop_y2 = min(img_h, cy + half_size)
        # Apply mask and replace background with flat gray (127, 127, 127) for TripoSR
        img_np = np.array(img_pil)  # RGB
        gray_bg = np.full_like(img_np, 127)
        mask_3d = mask_np[:, :, np.newaxis]
        img_masked = np.where(mask_3d > 0, img_np, gray_bg)
        
        # Crop the masked image
        img_crop = Image.fromarray(img_masked).crop((crop_x1, crop_y1, crop_x2, crop_y2))
        
        # Publish
        try:
            cv_crop = np.array(img_crop) # Already RGB from PIL
            msg = self.bridge.cv2_to_imgmsg(cv_crop, encoding='rgb8')
            self.triposr_img_pub.publish(msg)
            self.get_logger().info("Published crop image for TripoSR.")
            
            # Call service
            if not self.mesh_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("TripoSR service not available, skipping mesh generation.")
                return
                
            req = Trigger.Request()
            future = self.mesh_client.call_async(req)
            future.add_done_callback(self._mesh_service_cb)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing TripoSR crop: {e}")
            
    def _mesh_service_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✨ TripoSR mesh generated: {response.message}")
            else:
                self.get_logger().error(f"TripoSR mesh failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def _reset_tracking(self):
        """Reset the tracking state for a fresh start."""
        self.tracking_active = False
        self.inference_state = None
        self.current_frame_idx = -1
        self.tracking_frame_count = 0
        self.new_frame_available = False
        # Clear GPU memory
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    def _mask_to_tf(self, mask_np, depth_map, score):
        """Compute 3D centroid from mask and publish TF. Returns (u, v)."""
        # Find centroid of the mask
        M = cv2.moments(mask_np)
        if M["m00"] == 0:
            return None
        
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        
        if hasattr(self, 'grasp_offset'):
            u += self.grasp_offset[0]
            v += self.grasp_offset[1]
        
        # Clamp to bounds
        v = min(max(0, v), depth_map.shape[0] - 1)
        u = min(max(0, u), depth_map.shape[1] - 1)
        
        # Get depth at centroid (with small window median)
        r = 3
        v_min, v_max = max(0, v - r), min(depth_map.shape[0], v + r + 1)
        u_min, u_max = max(0, u - r), min(depth_map.shape[1], u + r + 1)
        depth_patch = depth_map[v_min:v_max, u_min:u_max]
        
        valid = depth_patch[(depth_patch > 0.01) & (depth_patch < 10.0) & np.isfinite(depth_patch)]
        if len(valid) == 0:
            return (u, v)
        
        Z = float(np.median(valid))
        
        # Back-project
        fx, fy, cx, cy = self.camera_intrinsics
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        
        # Transform to odom
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'hand_cam', rclpy.time.Time()
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            rx, ry, rz, rw = (t.transform.rotation.x, t.transform.rotation.y, 
                              t.transform.rotation.z, t.transform.rotation.w)
            
            # Rotate point
            # ... reuse _rotate_point_by_quaternion static method ...
            px, py, pz = self._rotate_point_by_quaternion(X, Y, Z, rx, ry, rz, rw)
            
            target_x = px + tx
            target_y = py + ty
            target_z = pz + tz
            
            # Publish TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = self.target_frame_name
            tf_msg.transform.translation.x = target_x
            tf_msg.transform.translation.y = target_y
            tf_msg.transform.translation.z = target_z
            tf_msg.transform.rotation.w = 1.0
            
            self.target_transform = tf_msg
            self.tf_broadcaster.sendTransform(tf_msg)
            
        except Exception:
            pass
            
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
            self.tf_broadcaster.sendTransform(self.target_transform)


def main(args=None):
    """ROS 2 node entry point."""
    rclpy.init(args=args)
    node = DetectQwenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def cli_main():
    """CLI mode for debugging (original behavior)."""
    if len(sys.argv) < 2:
        print("Uso:")
        print("  1. Camera: python3 detect_qwen.py --cli \"<prompt_do_objeto>\"")
        print("  2. Arquivo: python3 detect_qwen.py --cli <caminho_da_imagem> \"<prompt_do_objeto>\"")
        sys.exit(1)

    args = [a for a in sys.argv[1:] if a != '--cli']
    
    if len(args) == 1:
        mode = "camera"
        image_source = None
        object_prompt = args[0]
        output_filename = "camera_capture_detected.jpg"
    elif len(args) >= 2:
        mode = "file"
        image_source = args[0]
        object_prompt = args[1]
        p = Path(image_source)
        output_filename = str(p.with_name(f"{p.stem}_detected{p.suffix}"))
    else:
        print("Argumentos insuficientes.")
        sys.exit(1)
    
    print(f"Modo: {mode}")
    print(f"Detectando: '{object_prompt}'")
    
    img_pil = None
    if mode == "file":
        if not Path(image_source).exists():
             print(f"Erro: Imagem não encontrada: {image_source}")
             sys.exit(1)
        print(f"Carregando arquivo: {image_source}")
        img_pil = Image.open(image_source)
    else:
        print("Iniciando captura da camera...")
        img_pil = capture_image_from_ros()
        print("Imagem capturada com sucesso.")

    orig_w, orig_h = img_pil.size
    print(f"Original Image Size: {orig_w}x{orig_h}")
    
    max_dim = 1024
    scale_factor = max_dim / max(orig_w, orig_h)
    new_w = int(orig_w * scale_factor)
    new_h = int(orig_h * scale_factor)
    img_resized = img_pil.resize((new_w, new_h), Image.LANCZOS)
    
    print(f"Resized input from {orig_w}x{orig_h} to: {new_w}x{new_h} (Scale: {scale_factor:.3f})")
    
    try:
        response_text = detect_object(img_resized, object_prompt, VLLM_URL)
        print(f"Resposta Raw:\n{response_text}\n")
        
        boxes = parse_qwen_response(response_text, image_size=(new_w, new_h))
        
        if boxes:
            draw_result(img_pil, boxes, output_filename)
            
            if SAM2_AVAILABLE:
                p_out = Path(output_filename)
                seg_out_name = str(p_out.with_name(f"{p_out.stem.replace('_detected', '')}_segmented{p_out.suffix}"))
                segment_result(img_pil, boxes, seg_out_name)
                
        else:
            print("Nenhuma bounding box encontrada no formato esperado.")
            
    except Exception as e:
        print(f"Erro durante deteccao: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    if '--cli' in sys.argv:
        cli_main()
    else:
        # Default: run as ROS 2 node
        main()


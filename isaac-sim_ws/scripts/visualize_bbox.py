#!/usr/bin/env python3
"""
Script simples para visualizar bounding boxes COCO em uma imagem.

Uso:
    python visualize_bbox.py
    python visualize_bbox.py --image path/to/image.png
"""

import json
import argparse
from pathlib import Path
import sys

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Erro: PIL não instalado")
    print("Instale com: python3 -m pip install Pillow")
    sys.exit(1)


def visualize_coco_bbox(image_path, coco_json_path, output_path=None):
    """Visualiza bounding boxes COCO em uma imagem."""
    
    # Carrega COCO JSON
    print(f"📂 Carregando annotations: {coco_json_path}")
    with open(coco_json_path, 'r') as f:
        coco_data = json.load(f)
    
    # Cria mapeamentos
    categories = {cat['id']: cat['name'] for cat in coco_data['categories']}
    images = {img['id']: img for img in coco_data['images']}
    
    # Encontra a imagem
    image_name = Path(image_path).name
    image_folder = Path(image_path).parent.name
    relative_path = f"{image_folder}/{image_name}"
    
    image_id = None
    for img_id, img_info in images.items():
        if img_info['file_name'] == relative_path:
            image_id = img_id
            break
    
    if image_id is None:
        print(f"❌ Imagem não encontrada no COCO: {relative_path}")
        return False
    
    # Encontra annotations para esta imagem
    annotations = [ann for ann in coco_data['annotations'] if ann['image_id'] == image_id]
    
    print(f"🖼️  Imagem: {relative_path}")
    print(f"📊 Encontradas {len(annotations)} bounding boxes")
    
    # Carrega imagem
    img = Image.open(image_path)
    draw = ImageDraw.Draw(img)
    
    # Cores para cada classe
    colors = ['red', 'green', 'blue', 'yellow', 'magenta', 'cyan', 'orange', 'purple']
    
    # Tenta carregar fonte
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    except:
        font = ImageFont.load_default()
    
    # Desenha cada bbox
    for idx, ann in enumerate(annotations):
        bbox = ann['bbox']  # [x, y, width, height]
        x, y, w, h = bbox
        
        cat_id = ann['category_id']
        cat_name = categories.get(cat_id, 'unknown')
        color = colors[cat_id % len(colors)]
        
        # Desenha retângulo
        draw.rectangle([x, y, x + w, y + h], outline=color, width=4)
        
        # Desenha label
        label = f"{cat_name} ({ann['id']})"
        
        # Bounding box do texto
        bbox_text = draw.textbbox((x, y), label, font=font)
        text_width = bbox_text[2] - bbox_text[0]
        text_height = bbox_text[3] - bbox_text[1]
        
        # Background do label
        draw.rectangle([x, y - text_height - 8, x + text_width + 8, y], fill=color)
        draw.text((x + 4, y - text_height - 4), label, fill='white', font=font)
        
        print(f"  ✓ {cat_name}: bbox=[{x:.1f}, {y:.1f}, {w:.1f}, {h:.1f}]")
    
    # Salva ou mostra
    if output_path:
        img.save(output_path)
        print(f"\n💾 Salvo em: {output_path}")
    else:
        output_path = Path(image_path).parent / f"{Path(image_path).stem}_bbox.png"
        img.save(output_path)
        print(f"\n💾 Salvo em: {output_path}")
    
    # Tenta abrir
    try:
        img.show()
        print("🖼️  Imagem aberta!")
    except:
        print("ℹ️  Não foi possível abrir a imagem automaticamente")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Visualizar bounding boxes COCO")
    parser.add_argument(
        "--image",
        type=str,
        default="/workspace/datasets/valve_wheel_coco/images/Replicator/rgb_0000.png",
        help="Caminho da imagem"
    )
    parser.add_argument(
        "--coco",
        type=str,
        default="/workspace/datasets/valve_wheel_coco/train.json",
        help="Caminho do COCO JSON"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Caminho para salvar (opcional)"
    )
    
    args = parser.parse_args()
    
    # Verifica se arquivos existem
    if not Path(args.image).exists():
        print(f"❌ Imagem não encontrada: {args.image}")
        return 1
    
    if not Path(args.coco).exists():
        print(f"❌ COCO JSON não encontrado: {args.coco}")
        print("\n💡 Você executou o conversor?")
        print("   python3 isaac_to_coco_yolo.py")
        return 1
    
    print("=" * 70)
    print("🔍 VISUALIZADOR DE BOUNDING BOXES COCO")
    print("=" * 70)
    
    success = visualize_coco_bbox(args.image, args.coco, args.output)
    
    if success:
        print("\n✅ Visualização concluída!")
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())

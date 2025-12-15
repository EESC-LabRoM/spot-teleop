#!/usr/bin/env python3
"""
Visualizador interativo de bounding boxes COCO com OpenCV.

Navegue entre as imagens do dataset e veja as bounding boxes.

Controles:
    ESPAÇO / → : Próxima imagem
    ← : Imagem anterior
    R : Imagem aleatória
    S : Salvar imagem atual
    Q / ESC : Sair

Uso:
    python visualize_interactive.py
    python visualize_interactive.py --coco train.json
"""

import json
import argparse
import random
from pathlib import Path
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("Erro: OpenCV não instalado")
    print("Instale com: python3 -m pip install opencv-python")
    sys.exit(1)


class CocoViewer:
    """Visualizador interativo de dataset COCO."""
    
    def __init__(self, coco_json_path, images_dir):
        self.coco_json_path = Path(coco_json_path)
        self.images_dir = Path(images_dir)
        
        # Carrega COCO data
        print(f"📂 Carregando: {self.coco_json_path}")
        with open(self.coco_json_path, 'r') as f:
            self.coco_data = json.load(f)
        
        # Cria mapeamentos
        self.categories = {cat['id']: cat['name'] for cat in self.coco_data['categories']}
        self.images = {img['id']: img for img in self.coco_data['images']}
        
        # Agrupa annotations por imagem
        self.annotations_by_image = {}
        for ann in self.coco_data['annotations']:
            img_id = ann['image_id']
            if img_id not in self.annotations_by_image:
                self.annotations_by_image[img_id] = []
            self.annotations_by_image[img_id].append(ann)
        
        # Lista de IDs de imagens com annotations
        self.image_ids = sorted(self.annotations_by_image.keys())
        self.current_idx = 0
        
        # Cores para cada categoria (BGR para OpenCV)
        self.colors = {
            1: (0, 0, 255),    # Vermelho
            2: (0, 255, 0),    # Verde
            3: (255, 0, 0),    # Azul
            4: (0, 255, 255),  # Amarelo
            5: (255, 0, 255),  # Magenta
            6: (255, 255, 0),  # Ciano
        }
        
        print(f"✓ {len(self.images)} imagens")
        print(f"✓ {len(self.image_ids)} imagens com annotations")
        print(f"✓ {len(self.coco_data['annotations'])} annotations")
        print(f"✓ {len(self.categories)} categorias")
        print()
    
    def get_color(self, category_id):
        """Retorna cor para a categoria."""
        return self.colors.get(category_id, (0, 165, 255))  # Laranja default
    
    def draw_bboxes(self, image, annotations):
        """Desenha bounding boxes na imagem."""
        img_draw = image.copy()
        
        for ann in annotations:
            bbox = ann['bbox']  # [x, y, width, height]
            x, y, w, h = [int(v) for v in bbox]
            
            cat_id = ann['category_id']
            cat_name = self.categories.get(cat_id, 'unknown')
            color = self.get_color(cat_id)
            
            # Desenha retângulo
            cv2.rectangle(img_draw, (x, y), (x + w, y + h), color, 3)
            
            # Prepara label
            label = f"{cat_name}"
            
            # Calcula tamanho do texto
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            thickness = 2
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, thickness)
            
            # Desenha background do label
            cv2.rectangle(img_draw, 
                         (x, y - text_h - 10), 
                         (x + text_w + 10, y), 
                         color, -1)
            
            # Desenha texto
            cv2.putText(img_draw, label, 
                       (x + 5, y - 5), 
                       font, font_scale, (255, 255, 255), thickness)
        
        return img_draw
    
    def draw_hud(self, image, img_info, annotations):
        """Desenha HUD com informações."""
        h, w = image.shape[:2]
        
        # Background semi-transparente para o HUD
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # Informações
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Linha 1: Imagem
        text = f"Imagem: {self.current_idx + 1}/{len(self.image_ids)} - {img_info['file_name']}"
        cv2.putText(image, text, (10, 25), font, 0.6, (255, 255, 255), 2)
        
        # Linha 2: Dimensões e annotations
        text = f"Dimensoes: {img_info['width']}x{img_info['height']} | Bboxes: {len(annotations)}"
        cv2.putText(image, text, (10, 50), font, 0.6, (255, 255, 255), 2)
        
        # Linha 3: Classes
        class_counts = {}
        for ann in annotations:
            cat_name = self.categories.get(ann['category_id'], 'unknown')
            class_counts[cat_name] = class_counts.get(cat_name, 0) + 1
        
        classes_text = " | ".join([f"{name}: {count}" for name, count in class_counts.items()])
        cv2.putText(image, classes_text, (10, 75), font, 0.6, (0, 255, 255), 2)
        
        # Linha 4: Controles
        controls = "ESPACO/->: Proxima | <-: Anterior | R: Random | S: Salvar | Q/ESC: Sair"
        cv2.putText(image, controls, (10, 100), font, 0.5, (150, 150, 150), 1)
        
        return image
    
    def load_and_display(self):
        """Carrega e exibe imagem atual."""
        img_id = self.image_ids[self.current_idx]
        img_info = self.images[img_id]
        annotations = self.annotations_by_image.get(img_id, [])
        
        # Carrega imagem
        img_path = self.images_dir / img_info['file_name']
        
        if not img_path.exists():
            print(f"⚠️  Imagem não encontrada: {img_path}")
            return None
        
        image = cv2.imread(str(img_path))
        if image is None:
            print(f"⚠️  Erro ao carregar: {img_path}")
            return None
        
        # Desenha bboxes
        image = self.draw_bboxes(image, annotations)
        
        # Desenha HUD
        image = self.draw_hud(image, img_info, annotations)
        
        return image, img_info
    
    def save_current(self):
        """Salva imagem atual."""
        img_id = self.image_ids[self.current_idx]
        img_info = self.images[img_id]
        
        result = self.load_and_display()
        if result is None:
            return
        
        image, _ = result
        
        output_path = Path(f"bbox_viewer_output_{self.current_idx:04d}.png")
        cv2.imwrite(str(output_path), image)
        print(f"💾 Salvo: {output_path}")
    
    def run(self):
        """Loop principal do visualizador."""
        window_name = "COCO Dataset Viewer - Bounding Boxes"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        
        print("🚀 Visualizador iniciado!")
        print()
        print("Controles:")
        print("  ESPAÇO / → : Próxima imagem")
        print("  ← : Imagem anterior")
        print("  R : Imagem aleatória")
        print("  S : Salvar imagem atual")
        print("  Q / ESC : Sair")
        print()
        
        while True:
            result = self.load_and_display()
            if result is None:
                print("Pulando imagem...")
                self.current_idx = (self.current_idx + 1) % len(self.image_ids)
                continue
            
            image, img_info = result
            
            cv2.imshow(window_name, image)
            
            # Aguarda tecla (1ms)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # Q ou ESC
                print("\n👋 Saindo...")
                break
            
            elif key == ord(' ') or key == 83:  # ESPAÇO ou seta direita
                self.current_idx = (self.current_idx + 1) % len(self.image_ids)
                print(f"→ Próxima ({self.current_idx + 1}/{len(self.image_ids)})")
            
            elif key == 81:  # Seta esquerda
                self.current_idx = (self.current_idx - 1) % len(self.image_ids)
                print(f"← Anterior ({self.current_idx + 1}/{len(self.image_ids)})")
            
            elif key == ord('r') or key == ord('R'):  # R - Random
                self.current_idx = random.randint(0, len(self.image_ids) - 1)
                print(f"🎲 Aleatória ({self.current_idx + 1}/{len(self.image_ids)})")
            
            elif key == ord('s') or key == ord('S'):  # S - Save
                self.save_current()
        
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Visualizador interativo de bounding boxes COCO"
    )
    parser.add_argument(
        "--coco",
        type=str,
        default="/workspace/datasets/valve_wheel_coco/train.json",
        help="Caminho do COCO JSON"
    )
    parser.add_argument(
        "--images",
        type=str,
        default="/workspace/datasets/valve_wheel_coco/images",
        help="Diretório das imagens"
    )
    
    args = parser.parse_args()
    
    # Verifica arquivos
    if not Path(args.coco).exists():
        print(f"❌ COCO JSON não encontrado: {args.coco}")
        return 1
    
    if not Path(args.images).exists():
        print(f"❌ Diretório de imagens não encontrado: {args.images}")
        return 1
    
    print("=" * 70)
    print("🔍 VISUALIZADOR INTERATIVO DE BOUNDING BOXES")
    print("=" * 70)
    print()
    
    # Cria e executa visualizador
    viewer = CocoViewer(args.coco, args.images)
    viewer.run()
    
    print("\n✅ Finalizado!")
    return 0


if __name__ == "__main__":
    sys.exit(main())

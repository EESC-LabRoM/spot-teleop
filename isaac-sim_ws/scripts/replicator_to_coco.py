#!/usr/bin/env python3
"""
Converte dataset Isaac Sim Replicator para formato COCO e prepara para treino YOLO.
YOLOv11 aceita COCO diretamente - não precisa converter!

Uso:
    python isaac_to_coco_yolo.py
    python isaac_to_coco_yolo.py --input /path/to/dataset --output /path/to/output
"""

import argparse
import json
import os
from pathlib import Path
from typing import Dict, List, Tuple
import sys
import random

try:
    import numpy as np
    from PIL import Image
except ImportError as e:
    print(f"Erro: Biblioteca faltando: {e}")
    print("Instale com: pip install numpy pillow")
    sys.exit(1)


class IsaacSimToCOCO:
    """Converte Isaac Sim Replicator para COCO (compatível com YOLO v11)."""
    
    def __init__(self, input_dir: str, output_dir: str, split_ratio: Dict[str, float] = None):
        self.input_dir = Path(input_dir)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Split padrão: 80% train, 20% val
        self.split_ratio = split_ratio or {'train': 0.8, 'val': 0.2}
        
        # Estrutura COCO base
        self.coco_base = {
            "info": {
                "description": "Valve Wheel Dataset - Isaac Sim",
                "version": "1.0",
                "year": 2025,
                "contributor": "Isaac Sim Replicator",
                "date_created": "2025-10-26"
            },
            "licenses": [],
            "categories": []
        }
        
        self.category_map = {}
        self.image_id = 0
        self.annotation_id = 0
        
    def load_labels(self, labels_file: Path) -> Dict[str, int]:
        """Carrega labels do arquivo JSON."""
        with open(labels_file, 'r') as f:
            labels_data = json.load(f)
        
        label_map = {}
        for label_id, label_info in labels_data.items():
            class_name = label_info.get('class', f'class_{label_id}')
            label_map[label_id] = class_name
            
            # Adiciona às categorias se não existir
            if class_name not in self.category_map:
                cat_id = len(self.category_map) + 1
                self.category_map[class_name] = cat_id
        
        return label_map
    
    def load_bounding_boxes(self, bbox_file: Path) -> np.ndarray:
        """Carrega bounding boxes do arquivo numpy."""
        try:
            return np.load(bbox_file)
        except Exception as e:
            print(f"Aviso: Não foi possível ler {bbox_file}: {e}")
            return np.array([])
    
    def get_image_dimensions(self, image_path: Path) -> Tuple[int, int]:
        """Obtém dimensões da imagem."""
        try:
            with Image.open(image_path) as img:
                return img.size  # (width, height)
        except Exception as e:
            print(f"Aviso: Erro ao ler imagem {image_path}: {e}")
            return (0, 0)
    
    def process_camera_folder(self, camera_folder: Path, camera_name: str, 
                             images_data: List, annotations_data: List):
        """Processa todas as imagens de uma pasta de câmera."""
        print(f"\nProcessando câmera: {camera_name}")
        
        rgb_folder = camera_folder / "rgb"
        bbox_folder = camera_folder / "bounding_box_2d_tight"
        
        if not rgb_folder.exists() or not bbox_folder.exists():
            print(f"Aviso: Pastas rgb ou bbox não encontradas em {camera_folder}")
            return
        
        image_files = sorted(rgb_folder.glob("rgb_*.png"))
        print(f"Encontradas {len(image_files)} imagens")
        
        for img_file in image_files:
            frame_num = img_file.stem.split('_')[1]
            
            bbox_file = bbox_folder / f"bounding_box_2d_tight_{frame_num}.npy"
            labels_file = bbox_folder / f"bounding_box_2d_tight_labels_{frame_num}.json"
            
            if not bbox_file.exists() or not labels_file.exists():
                continue
            
            width, height = self.get_image_dimensions(img_file)
            if width == 0 or height == 0:
                continue
            
            self.image_id += 1
            relative_path = f"{camera_name}/{img_file.name}"
            
            # Adiciona info da imagem
            image_info = {
                "id": self.image_id,
                "file_name": relative_path,
                "width": width,
                "height": height,
                "license": 0,
                "flickr_url": "",
                "coco_url": "",
                "date_captured": ""
            }
            images_data.append(image_info)
            
            # Carrega labels e bboxes
            label_map = self.load_labels(labels_file)
            bboxes = self.load_bounding_boxes(bbox_file)
            
            if len(bboxes) == 0:
                continue
            
            # Processa cada bbox
            for bbox_idx, bbox in enumerate(bboxes):
                # Isaac Sim retorna structured array com campos nomeados
                # Acessa corretamente: x_min, y_min, x_max, y_max
                try:
                    x_min = float(bbox['x_min'])
                    y_min = float(bbox['y_min'])
                    x_max = float(bbox['x_max'])
                    y_max = float(bbox['y_max'])
                except (KeyError, ValueError):
                    # Fallback para formato antigo (se existir)
                    continue
                
                # Converte bbox: [x_min, y_min, x_max, y_max] -> [x, y, width, height]
                w = x_max - x_min
                h = y_max - y_min
                
                if w <= 0 or h <= 0:
                    continue
                
                # Mapeia classe usando semanticId do bbox
                try:
                    semantic_id = str(int(bbox['semanticId']))
                except (KeyError, ValueError):
                    semantic_id = str(bbox_idx)
                
                class_id_str = semantic_id if semantic_id in label_map else "0"
                class_name = label_map.get(class_id_str, "unknown")
                category_id = self.category_map.get(class_name, 1)
                
                self.annotation_id += 1
                annotation = {
                    "id": self.annotation_id,
                    "image_id": self.image_id,
                    "category_id": category_id,
                    "bbox": [float(x_min), float(y_min), float(w), float(h)],
                    "area": float(w * h),
                    "segmentation": [],
                    "iscrowd": 0
                }
                annotations_data.append(annotation)
    
    def convert(self):
        """Processo principal de conversão."""
        print("=" * 70)
        print("CONVERSOR ISAAC SIM → COCO (para YOLO v11)")
        print("=" * 70)
        print(f"Input:  {self.input_dir}")
        print(f"Output: {self.output_dir}")
        
        # Encontra pastas de câmeras
        camera_folders = []
        for folder_name in ["Replicator", "Replicator_01", "Replicator_02", "Replicator_03"]:
            folder_path = self.input_dir / folder_name
            if folder_path.exists():
                camera_folders.append((folder_path, folder_name))
        
        if not camera_folders:
            print("Erro: Nenhuma pasta de câmera encontrada!")
            return False
        
        print(f"\nEncontradas {len(camera_folders)} câmeras")
        
        # Coleta todas as imagens e annotations
        all_images = []
        all_annotations = []
        
        for camera_path, camera_name in camera_folders:
            self.process_camera_folder(camera_path, camera_name, all_images, all_annotations)
        
        # Atualiza categorias na estrutura base
        for cat_name, cat_id in sorted(self.category_map.items(), key=lambda x: x[1]):
            self.coco_base["categories"].append({
                "id": cat_id,
                "name": cat_name,
                "supercategory": "valve_component"
            })
        
        # Split train/val
        print(f"\nDividindo dataset...")
        total_images = len(all_images)
        random.seed(42)
        random.shuffle(all_images)
        
        train_count = int(total_images * self.split_ratio['train'])
        train_images = all_images[:train_count]
        val_images = all_images[train_count:]
        
        train_image_ids = {img['id'] for img in train_images}
        val_image_ids = {img['id'] for img in val_images}
        
        train_annotations = [ann for ann in all_annotations if ann['image_id'] in train_image_ids]
        val_annotations = [ann for ann in all_annotations if ann['image_id'] in val_image_ids]
        
        print(f"  Train: {len(train_images)} imagens, {len(train_annotations)} annotations")
        print(f"  Val:   {len(val_images)} imagens, {len(val_annotations)} annotations")
        
        # Cria estrutura de diretórios
        images_dir = self.output_dir / "images"
        images_dir.mkdir(exist_ok=True)
        
        # Copia/symlink imagens
        print(f"\nCopiando imagens...")
        for camera_path, camera_name in camera_folders:
            rgb_folder = camera_path / "rgb"
            output_camera_dir = images_dir / camera_name
            output_camera_dir.mkdir(exist_ok=True)
            
            for img_file in rgb_folder.glob("rgb_*.png"):
                output_path = output_camera_dir / img_file.name
                if not output_path.exists():
                    try:
                        output_path.symlink_to(img_file.absolute())
                    except:
                        import shutil
                        shutil.copy2(img_file, output_path)
        
        # Salva JSONs COCO
        train_coco = {**self.coco_base, "images": train_images, "annotations": train_annotations}
        val_coco = {**self.coco_base, "images": val_images, "annotations": val_annotations}
        
        train_json = self.output_dir / "train.json"
        val_json = self.output_dir / "val.json"
        
        print(f"\nSalvando annotations...")
        with open(train_json, 'w') as f:
            json.dump(train_coco, f, indent=2)
        print(f"  ✓ {train_json}")
        
        with open(val_json, 'w') as f:
            json.dump(val_coco, f, indent=2)
        print(f"  ✓ {val_json}")
        
        # Cria arquivo YAML para YOLO
        self.create_yolo_config()
        
        # Resumo
        print("\n" + "=" * 70)
        print("CONVERSÃO CONCLUÍDA! ✓")
        print("=" * 70)
        print(f"Total de imagens:     {total_images}")
        print(f"Total de annotations: {len(all_annotations)}")
        print(f"Classes:              {len(self.category_map)}")
        print("\nClasses encontradas:")
        for cat_name, cat_id in sorted(self.category_map.items(), key=lambda x: x[1]):
            count = sum(1 for ann in all_annotations if ann['category_id'] == cat_id)
            print(f"  {cat_id}: {cat_name} ({count} objetos)")
        
        print(f"\n📁 Output: {self.output_dir}")
        print(f"  ├── train.json ({len(train_images)} imagens)")
        print(f"  ├── val.json ({len(val_images)} imagens)")
        print(f"  ├── images/")
        print(f"  └── data.yaml")
        
        print("\n🚀 Próximo passo - Treinar YOLO:")
        print(f"   yolo detect train data={self.output_dir / 'data.yaml'} model=yolo11n.pt epochs=100")
        print("=" * 70)
        
        return True
    
    def create_yolo_config(self):
        """Cria arquivo de configuração YAML para YOLO."""
        yaml_path = self.output_dir / "data.yaml"
        
        # Nomes das classes em ordem
        class_names = [cat_name for cat_name, _ in sorted(self.category_map.items(), 
                                                           key=lambda x: self.category_map[x[0]])]
        
        yaml_content = f"""# YOLO v11 Dataset Configuration
# Formato: COCO (nativo)
# Dataset: valve_wheel - Isaac Sim

path: {self.output_dir.absolute()}  # Caminho do dataset
train: train.json  # Annotations de treino (COCO format)
val: val.json      # Annotations de validação (COCO format)

# Classes
names:
"""
        
        for idx, name in enumerate(class_names):
            yaml_content += f"  {idx}: {name}\n"
        
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)
        
        print(f"\n✓ Configuração YOLO criada: {yaml_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Converte Isaac Sim para COCO (compatível com YOLO v11)"
    )
    parser.add_argument(
        "--input",
        type=str,
        default="/workspace/datasets/valve_wheel",
        help="Diretório do dataset Isaac Sim"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="/workspace/datasets/valve_wheel_coco",
        help="Diretório de saída (formato COCO)"
    )
    parser.add_argument(
        "--train-split",
        type=float,
        default=0.8,
        help="Proporção de dados para treino (padrão: 0.8)"
    )
    
    args = parser.parse_args()
    
    split_ratio = {
        'train': args.train_split,
        'val': 1.0 - args.train_split
    }
    
    converter = IsaacSimToCOCO(args.input, args.output, split_ratio)
    success = converter.convert()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

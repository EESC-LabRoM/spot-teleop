#!/usr/bin/env python3
"""
Converte dataset COCO para formato YOLO manualmente.
Solução alternativa que não depende de bugs na função convert_coco.

Uso:
    python coco_to_yolo_manual.py
    python coco_to_yolo_manual.py --input /path/to/coco --output /path/to/yolo
"""

import argparse
import json
import shutil
import sys
from pathlib import Path
from collections import defaultdict


def coco_to_yolo_bbox(bbox, img_width, img_height):
    """
    Converte bbox COCO [x, y, width, height] para YOLO [x_center, y_center, width, height] normalizado.
    """
    x, y, w, h = bbox
    
    # Converte para centro
    x_center = x + w / 2
    y_center = y + h / 2
    
    # Normaliza
    x_center_norm = x_center / img_width
    y_center_norm = y_center / img_height
    w_norm = w / img_width
    h_norm = h / img_height
    
    # Garante que os valores estão entre 0 e 1
    x_center_norm = max(0.0, min(1.0, x_center_norm))
    y_center_norm = max(0.0, min(1.0, y_center_norm))
    w_norm = max(0.0, min(1.0, w_norm))
    h_norm = max(0.0, min(1.0, h_norm))
    
    return [x_center_norm, y_center_norm, w_norm, h_norm]


def convert_coco_to_yolo(input_dir, output_dir, split='train'):
    """
    Converte um split (train ou val) do COCO para YOLO.
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    json_file = input_path / f"{split}.json"
    if not json_file.exists():
        print(f"Aviso: {json_file} não encontrado, pulando {split}...")
        return
    
    print(f"\n{'='*60}")
    print(f"Convertendo {split}...")
    print(f"{'='*60}")
    
    # Carrega o JSON COCO
    with open(json_file, 'r') as f:
        coco_data = json.load(f)
    
    # Cria mapa de categorias
    category_map = {}
    for cat in coco_data.get('categories', []):
        category_map[cat['id']] = cat
    
    print(f"Categorias encontradas: {len(category_map)}")
    for cat_id, cat in category_map.items():
        print(f"  {cat_id}: {cat['name']}")
    
    # Cria mapa de imagens
    image_map = {}
    for img in coco_data.get('images', []):
        image_map[img['id']] = img
    
    print(f"Imagens: {len(image_map)}")
    
    # Agrupa anotações por imagem
    annotations_by_image = defaultdict(list)
    for ann in coco_data.get('annotations', []):
        annotations_by_image[ann['image_id']].append(ann)
    
    print(f"Anotações: {len(coco_data.get('annotations', []))}")
    
    # Processa cada imagem
    images_dir = output_path / 'images' / split
    labels_dir = output_path / 'labels' / split
    
    converted_count = 0
    skipped_count = 0
    
    for img_id, img_info in image_map.items():
        file_name = img_info['file_name']
        img_width = img_info['width']
        img_height = img_info['height']
        
        # Caminho da imagem de origem
        src_img_path = input_path / 'images' / file_name
        
        if not src_img_path.exists():
            print(f"Aviso: Imagem não encontrada: {src_img_path}")
            skipped_count += 1
            continue
        
        # Cria estrutura de diretórios (preserva subdirs)
        file_path = Path(file_name)
        if file_path.parent != Path('.'):
            (images_dir / file_path.parent).mkdir(parents=True, exist_ok=True)
            (labels_dir / file_path.parent).mkdir(parents=True, exist_ok=True)
        
        # Copia imagem
        dst_img_path = images_dir / file_name
        if not dst_img_path.exists():
            shutil.copy2(src_img_path, dst_img_path)
        
        # Cria arquivo de label YOLO
        label_path = (labels_dir / file_name).with_suffix('.txt')
        
        annotations = annotations_by_image.get(img_id, [])
        
        with open(label_path, 'w') as f:
            for ann in annotations:
                category_id = ann['category_id']
                bbox = ann['bbox']
                
                # Converte bbox para formato YOLO
                yolo_bbox = coco_to_yolo_bbox(bbox, img_width, img_height)
                
                # Escreve no formato YOLO: class x_center y_center width height
                # YOLO usa class_id começando em 0
                yolo_class_id = category_id - 1 if category_id > 0 else 0
                
                line = f"{yolo_class_id} {yolo_bbox[0]:.6f} {yolo_bbox[1]:.6f} {yolo_bbox[2]:.6f} {yolo_bbox[3]:.6f}\n"
                f.write(line)
        
        converted_count += 1
        
        if converted_count % 500 == 0:
            print(f"  Processadas: {converted_count}/{len(image_map)}")
    
    print(f"\n✓ {split} concluído!")
    print(f"  Convertidas: {converted_count}")
    print(f"  Ignoradas: {skipped_count}")
    
    return category_map


def create_data_yaml(output_dir, train_categories, dataset_name='valve_wheel'):
    """
    Cria o arquivo data.yaml para YOLO.
    """
    output_path = Path(output_dir)
    yaml_path = output_path / 'data.yaml'
    
    # Obtém nomes das classes na ordem correta
    class_names = []
    max_id = max(train_categories.keys()) if train_categories else 0
    
    for i in range(max_id):
        class_id = i + 1
        if class_id in train_categories:
            class_names.append(train_categories[class_id]['name'])
        else:
            class_names.append(f'class_{class_id}')
    
    yaml_content = f"""# YOLO Dataset Configuration
# Gerado automaticamente de COCO

path: {output_path.absolute()}  # dataset root dir
train: images/train  # train images (relative to 'path')
val: images/val  # val images (relative to 'path')

# Classes
names:
"""
    
    for idx, name in enumerate(class_names):
        yaml_content += f"  {idx}: {name}\n"
    
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"\n✓ Arquivo data.yaml criado: {yaml_path}")
    print(f"\nConteúdo:")
    print(yaml_content)


def main():
    parser = argparse.ArgumentParser(
        description="Converte dataset COCO para formato YOLO (manual)"
    )
    parser.add_argument(
        "--input",
        type=str,
        default="/workspace/datasets/valve_wheel_coco",
        help="Diretório de entrada COCO"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Diretório de saída YOLO (padrão: input_yolo)"
    )
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    if not input_dir.exists():
        print(f"Erro: Diretório não encontrado: {input_dir}")
        sys.exit(1)
    
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = input_dir.parent / f"{input_dir.name}_yolo"
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("="*60)
    print("Conversão COCO -> YOLO (Manual)")
    print("="*60)
    print(f"Entrada:  {input_dir}")
    print(f"Saída:    {output_dir}")
    print("="*60)
    
    # Converte train
    train_cats = convert_coco_to_yolo(input_dir, output_dir, 'train')
    
    # Converte val
    convert_coco_to_yolo(input_dir, output_dir, 'val')
    
    # Cria data.yaml
    if train_cats:
        create_data_yaml(output_dir, train_cats)
    
    print("\n" + "="*60)
    print("✓ Conversão concluída com sucesso!")
    print("="*60)
    print(f"\nDataset YOLO disponível em: {output_dir}")
    print("\nEstrutura criada:")
    print(f"  {output_dir}/")
    print(f"    ├── data.yaml")
    print(f"    ├── images/")
    print(f"    │   ├── train/")
    print(f"    │   └── val/")
    print(f"    └── labels/")
    print(f"        ├── train/")
    print(f"        └── val/")
    print("\nPara treinar com YOLO:")
    print(f"  yolo train data={output_dir}/data.yaml model=yolo11n.pt epochs=100")


if __name__ == "__main__":
    main()

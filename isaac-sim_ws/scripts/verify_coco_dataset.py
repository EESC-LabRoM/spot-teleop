#!/usr/bin/env python3
"""
Verify COCO format dataset and visualize samples.

Usage:
    python verify_coco_dataset.py --coco-json /path/to/annotations.json
    python verify_coco_dataset.py --coco-json /path/to/annotations.json --visualize --num-samples 5
"""

import argparse
import json
import random
from pathlib import Path
import sys

try:
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:
    print(f"Error: Missing required library: {e}")
    print("Please install: pip install numpy pillow")
    sys.exit(1)


class CocoDatasetVerifier:
    """Verify and visualize COCO format dataset."""
    
    def __init__(self, coco_json: str):
        self.coco_json = Path(coco_json)
        
        # Load COCO data
        print(f"Loading COCO JSON: {self.coco_json}")
        with open(self.coco_json, 'r') as f:
            self.coco_data = json.load(f)
        
        # Create mappings
        self.cat_id_to_name = {cat['id']: cat['name'] for cat in self.coco_data['categories']}
        self.image_id_to_info = {img['id']: img for img in self.coco_data['images']}
        
        # Group annotations by image ID
        self.annotations_by_image = {}
        for ann in self.coco_data['annotations']:
            img_id = ann['image_id']
            if img_id not in self.annotations_by_image:
                self.annotations_by_image[img_id] = []
            self.annotations_by_image[img_id].append(ann)
    
    def verify(self) -> bool:
        """Verify the COCO dataset structure and data integrity."""
        print("\n" + "=" * 60)
        print("COCO Dataset Verification")
        print("=" * 60)
        
        # Check required fields
        required_fields = ['info', 'images', 'annotations', 'categories']
        for field in required_fields:
            if field not in self.coco_data:
                print(f"✗ Missing required field: {field}")
                return False
        print("✓ All required fields present")
        
        # Verify categories
        print(f"\nCategories: {len(self.coco_data['categories'])}")
        if len(self.coco_data['categories']) == 0:
            print("✗ No categories found")
            return False
        
        for cat in self.coco_data['categories']:
            if 'id' not in cat or 'name' not in cat:
                print(f"✗ Invalid category: {cat}")
                return False
            print(f"  - {cat['name']} (ID: {cat['id']})")
        print("✓ Categories valid")
        
        # Verify images
        print(f"\nImages: {len(self.coco_data['images'])}")
        if len(self.coco_data['images']) == 0:
            print("✗ No images found")
            return False
        
        invalid_images = 0
        for img in self.coco_data['images']:
            required_img_fields = ['id', 'file_name', 'width', 'height']
            if not all(field in img for field in required_img_fields):
                print(f"✗ Invalid image: {img.get('id', 'unknown')}")
                invalid_images += 1
        
        if invalid_images > 0:
            print(f"✗ {invalid_images} invalid images found")
            return False
        print("✓ All images valid")
        
        # Verify annotations
        print(f"\nAnnotations: {len(self.coco_data['annotations'])}")
        if len(self.coco_data['annotations']) == 0:
            print("⚠ Warning: No annotations found")
        
        invalid_annotations = 0
        invalid_bbox_count = 0
        invalid_category_count = 0
        
        for ann in self.coco_data['annotations']:
            # Check required fields
            required_ann_fields = ['id', 'image_id', 'category_id', 'bbox']
            if not all(field in ann for field in required_ann_fields):
                invalid_annotations += 1
                continue
            
            # Check bbox format
            bbox = ann['bbox']
            if len(bbox) != 4:
                invalid_bbox_count += 1
                continue
            
            x, y, w, h = bbox
            if w <= 0 or h <= 0:
                invalid_bbox_count += 1
                continue
            
            # Check category ID
            if ann['category_id'] not in self.cat_id_to_name:
                invalid_category_count += 1
                continue
            
            # Check image ID
            if ann['image_id'] not in self.image_id_to_info:
                invalid_annotations += 1
                continue
        
        if invalid_annotations > 0:
            print(f"✗ {invalid_annotations} invalid annotations found")
            return False
        if invalid_bbox_count > 0:
            print(f"✗ {invalid_bbox_count} invalid bounding boxes found")
            return False
        if invalid_category_count > 0:
            print(f"✗ {invalid_category_count} invalid category IDs found")
            return False
        
        print("✓ All annotations valid")
        
        # Statistics
        print("\n" + "=" * 60)
        print("Dataset Statistics")
        print("=" * 60)
        
        # Annotations per image
        anns_per_img = [len(anns) for anns in self.annotations_by_image.values()]
        if anns_per_img:
            print(f"Annotations per image:")
            print(f"  Min: {min(anns_per_img)}")
            print(f"  Max: {max(anns_per_img)}")
            print(f"  Mean: {np.mean(anns_per_img):.2f}")
            print(f"  Median: {np.median(anns_per_img):.2f}")
        
        # Images without annotations
        images_without_anns = len(self.coco_data['images']) - len(self.annotations_by_image)
        if images_without_anns > 0:
            print(f"\n⚠ Warning: {images_without_anns} images without annotations")
        
        # Category distribution
        print(f"\nCategory distribution:")
        cat_counts = {}
        for ann in self.coco_data['annotations']:
            cat_id = ann['category_id']
            cat_name = self.cat_id_to_name[cat_id]
            cat_counts[cat_name] = cat_counts.get(cat_name, 0) + 1
        
        for cat_name, count in sorted(cat_counts.items()):
            percentage = (count / len(self.coco_data['annotations'])) * 100
            print(f"  {cat_name}: {count} ({percentage:.1f}%)")
        
        # Image dimensions
        widths = [img['width'] for img in self.coco_data['images']]
        heights = [img['height'] for img in self.coco_data['images']]
        print(f"\nImage dimensions:")
        print(f"  Width: {min(widths)}x{max(widths)} (mean: {np.mean(widths):.0f})")
        print(f"  Height: {min(heights)}x{max(heights)} (mean: {np.mean(heights):.0f})")
        
        print("\n" + "=" * 60)
        print("✓ Dataset verification passed!")
        print("=" * 60)
        
        return True
    
    def visualize_samples(self, num_samples: int = 5, output_dir: str = None):
        """Visualize random samples from the dataset."""
        print(f"\nVisualizing {num_samples} random samples...")
        
        if output_dir:
            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)
        else:
            output_path = self.coco_json.parent / "visualizations"
            output_path.mkdir(parents=True, exist_ok=True)
        
        # Get images base directory
        images_base_dir = self.coco_json.parent / 'images'
        
        # Select random images with annotations
        images_with_anns = [img for img in self.coco_data['images'] 
                           if img['id'] in self.annotations_by_image]
        
        if len(images_with_anns) == 0:
            print("No images with annotations to visualize")
            return
        
        samples = random.sample(images_with_anns, min(num_samples, len(images_with_anns)))
        
        # Colors for each category
        colors = ['red', 'green', 'blue', 'yellow', 'magenta', 'cyan', 'orange', 'purple']
        cat_colors = {cat['id']: colors[i % len(colors)] 
                     for i, cat in enumerate(self.coco_data['categories'])}
        
        for i, img_info in enumerate(samples):
            img_id = img_info['id']
            file_name = img_info['file_name']
            
            # Load image
            img_path = images_base_dir / file_name
            if not img_path.exists():
                print(f"Warning: Image not found: {img_path}")
                continue
            
            try:
                img = Image.open(img_path)
                draw = ImageDraw.Draw(img)
                
                # Draw annotations
                annotations = self.annotations_by_image.get(img_id, [])
                
                for ann in annotations:
                    bbox = ann['bbox']
                    x, y, w, h = bbox
                    cat_id = ann['category_id']
                    cat_name = self.cat_id_to_name[cat_id]
                    color = cat_colors[cat_id]
                    
                    # Draw bounding box
                    draw.rectangle([x, y, x + w, y + h], outline=color, width=3)
                    
                    # Draw label
                    label = f"{cat_name}"
                    
                    # Try to use a better font
                    try:
                        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
                    except:
                        font = ImageFont.load_default()
                    
                    # Get text bounding box
                    bbox_text = draw.textbbox((x, y), label, font=font)
                    text_width = bbox_text[2] - bbox_text[0]
                    text_height = bbox_text[3] - bbox_text[1]
                    
                    # Draw label background
                    draw.rectangle([x, y - text_height - 4, x + text_width + 4, y], fill=color)
                    draw.text((x + 2, y - text_height - 2), label, fill='white', font=font)
                
                # Save visualization
                output_file = output_path / f"sample_{i+1}_{Path(file_name).stem}.png"
                img.save(output_file)
                print(f"  Saved: {output_file}")
                
            except Exception as e:
                print(f"Error processing {img_path}: {e}")
        
        print(f"\n✓ Visualizations saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Verify and visualize COCO format dataset"
    )
    parser.add_argument(
        "--coco-json",
        type=str,
        required=True,
        help="Path to COCO annotations.json file"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Create visualization of sample images"
    )
    parser.add_argument(
        "--num-samples",
        type=int,
        default=5,
        help="Number of samples to visualize (default: 5)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        help="Output directory for visualizations (default: <coco_json_dir>/visualizations)"
    )
    
    args = parser.parse_args()
    
    # Create verifier
    verifier = CocoDatasetVerifier(args.coco_json)
    
    # Run verification
    if not verifier.verify():
        print("\n✗ Dataset verification failed!")
        return 1
    
    # Visualize if requested
    if args.visualize:
        verifier.visualize_samples(args.num_samples, args.output_dir)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

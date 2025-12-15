import omni.replicator.core as rep
import os
import glob
import asyncio

# ======== CONFIG ========
USD_PATH = "/workspace/valve_wheel_yellow.usd"
OUT_ROOT = "/workspace/datasets/valve_wheel"
RES      = (1280, 720)
N_TRAIN  = 30  # DEBUG: reduzido de 600
N_VAL    = 15   # DEBUG: reduzido de 150
SEED     = 42

def count_files(directory, pattern="*"):
    """Conta arquivos em um diretório."""
    if not os.path.exists(directory):
        return 0
    return len(glob.glob(os.path.join(directory, pattern)))

def validate_split_output(split_dir, expected_count, split_name):
    """Valida se os arquivos foram gerados corretamente."""
    import time
    
    # Retry com espera, pois I/O pode estar atrasado
    max_retries = 3
    for attempt in range(max_retries):
        # BasicWriter escreve diretamente no split_dir, não em subpastas
        rgb_count = count_files(split_dir, "rgb_*.png")
        seg_count = count_files(split_dir, "semantic_segmentation_*.png")
        bbox_json_count = count_files(split_dir, "bounding_box_2d_tight_labels_*.json")
        
        if rgb_count > 0:
            break
        
        if attempt < max_retries - 1:
            print(f"⏳ Tentativa {attempt + 1}/{max_retries}: aguardando arquivos aparecerem...")
            time.sleep(3)
    
    success = rgb_count > 0
    
    if success:
        print(f"✅ [{split_name}] Gerado com sucesso:")
        print(f"   - {rgb_count} imagens RGB")
        print(f"   - {seg_count} máscaras de segmentação")
        print(f"   - {bbox_json_count} arquivos JSON de bboxes")
    else:
        print(f"❌ [{split_name}] FALHOU - nenhum arquivo gerado!")
        print(f"   Esperado: {expected_count} frames")
        print(f"   Diretório verificado: {split_dir}")
        
    return success

async def warmup_replicator():
    """Aquece o Replicator com uma geração mínima para inicializar o sistema."""
    print("\n" + "="*60)
    print("🔥 WARMUP - Inicializando Replicator...")
    print("="*60)
    
    import tempfile
    temp_dir = tempfile.mkdtemp()
    
    try:
        with rep.new_layer():
            # Setup mínimo
            asset = rep.create.from_usd(USD_PATH, semantics=[("class", "valve_wheel")])
            cam = rep.create.camera(position=(0, 0, 1.0), look_at=asset)
            rp = rep.create.render_product(cam, RES)
            
            # Randomizador simples para warmup (inicializa o sistema de randomizadores)
            def warmup_randomizer():
                with asset:
                    rep.modify.pose(rotation=(0, 0, 0))
                return asset.node
            
            rep.randomizer.register(warmup_randomizer)
            
            # Writer simples
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=temp_dir, rgb=True)
            writer.attach([rp])
            
            # Gera apenas 1 frame COM randomizador
            with rep.trigger.on_frame(max_execs=1):
                rep.randomizer.warmup_randomizer()
            
            print("⚙️  Executando warmup (1 frame)...")
            await rep.orchestrator.run_async()
            
            # Aguarda writes completarem
            await asyncio.sleep(3)
            
            # Cleanup adequado
            writer.detach()
            rep.orchestrator.stop()
            await asyncio.sleep(2)
            
        print("✅ Warmup concluído - Sistema inicializado")
        
    except Exception as e:
        print(f"⚠️  Warmup teve erro (normal): {e}")
    
    # Limpa arquivos temporários
    import shutil
    try:
        shutil.rmtree(temp_dir)
    except:
        pass

async def make_split(split_outdir, n_frames, split_name):
    """Gera um split (train/val) com COCO + RGB + JSON de bboxes."""
    
    print(f"\n{'='*60}")
    print(f"Gerando split: {split_name}")
    print(f"Frames: {n_frames} | Output: {split_outdir}")
    print(f"{'='*60}")
    
    try:
        with rep.new_layer():
            # Carrega o ambiente warehouse (já vem com luzes)
            print("🏭 Carregando ambiente warehouse...")
            warehouse = rep.create.from_usd(
                "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Simple_Warehouse/warehouse.usd"
            )
            
            # Carrega o asset USD da válvula
            print("� Carregando válvula...")
            asset = rep.create.from_usd(
                USD_PATH, 
                semantics=[("class", "valve_wheel")],
                position=(0, 0, 1.0)  # Elevado do chão do warehouse
            )
            
            # Câmera + render product
            print("� Configurando câmera...")
            cam = rep.create.camera(
                position=(0, 0, 1.0),
                look_at=asset
            )
            rp = rep.create.render_product(cam, RES)
            
            # ===== RANDOMIZAÇÃO =====
            def randomize_scene():
                import math
                import random
                
                # Randomiza pose do objeto (centrado, pequena variação)
                with asset:
                    rep.modify.pose(
                        position=rep.distribution.uniform((-0.05, -0.05, -0.05), (0.05, 0.05, 0.05)),
                        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
                        scale=rep.distribution.uniform(0.95, 1.05)
                    )
                
                # Câmera orbita ao redor do objeto a uma distância fixa
                # Usa coordenadas esféricas para garantir visibilidade
                theta = random.uniform(0, 360)  # Ângulo horizontal
                phi = random.uniform(15, 75)    # Ângulo vertical (evita topo/fundo)
                distance = random.uniform(0.8, 1.5)  # Distância da câmera
                
                # Converte para coordenadas cartesianas
                theta_rad = math.radians(theta)
                phi_rad = math.radians(phi)
                
                cam_x = distance * math.sin(phi_rad) * math.cos(theta_rad)
                cam_y = distance * math.sin(phi_rad) * math.sin(theta_rad)
                cam_z = distance * math.cos(phi_rad)
                
                with cam:
                    rep.modify.pose(
                        position=(cam_x, cam_y, cam_z),
                        look_at=(0, 0, 0)  # Sempre olha para o centro onde está a válvula
                    )
                
                return asset.node
            
            print("🎲 Registrando randomizador...")
            rep.randomizer.register(randomize_scene)
            
            # ===== WRITERS =====
            print("💾 Configurando writers...")
            
            # Writer COCO
            coco_writer = rep.WriterRegistry.get("BasicWriter")
            coco_writer.initialize(
                output_dir=split_outdir,
                rgb=True,
                bounding_box_2d_tight=True,
                semantic_segmentation=True,
                colorize_semantic_segmentation=False
            )
            coco_writer.attach([rp])
            
            # ===== TRIGGER E EXECUÇÃO =====
            print(f"▶️  Iniciando geração de {n_frames} frames...")
            print(f"⏳ Renderizando... (isso pode demorar)")
            
            with rep.trigger.on_frame(max_execs=n_frames):
                rep.randomizer.randomize_scene()
            
            # EXECUTA O ORCHESTRATOR DE FORMA ASSÍNCRONA (para Kit)
            await rep.orchestrator.run_async()
            
            # CRÍTICO: Monitora arquivos sendo escritos até atingir o número esperado
            print(f"⏳ Aguardando geração de {n_frames} imagens...")
            timeout_seconds = n_frames * 2  # 2 segundos por frame como timeout
            elapsed = 0
            check_interval = 2  # Verifica a cada 2 segundos
            
            while elapsed < timeout_seconds:
                # Conta quantas imagens RGB foram geradas
                import glob
                rgb_files = glob.glob(os.path.join(split_outdir, "rgb_*.png"))
                current_count = len(rgb_files)
                
                if current_count >= n_frames:
                    print(f"✅ Todos os {n_frames} frames foram gerados!")
                    break
                
                # Log de progresso
                if elapsed % 10 == 0 and elapsed > 0:
                    print(f"   ... progresso: {current_count}/{n_frames} frames ({elapsed}s decorridos)")
                
                await asyncio.sleep(check_interval)
                elapsed += check_interval
            
            # Verifica se atingiu o número esperado ou timeout
            final_count = len(glob.glob(os.path.join(split_outdir, "rgb_*.png")))
            if final_count < n_frames:
                print(f"⚠️  Timeout: apenas {final_count}/{n_frames} frames gerados em {timeout_seconds}s")
            
            # Aguarda writes completarem
            await asyncio.sleep(3)
            
            # Detach writer ANTES de parar orchestrator
            print("🔌 Detaching writer...")
            coco_writer.detach()
            
            # Cleanup e aguarda finalização completa
            print("🧹 Limpando recursos...")
            rep.orchestrator.stop()
            
            # Aguarda I/O completar (crítico para grandes datasets)
            await asyncio.sleep(3)
            
        print(f"✓ Orchestrator finalizado para {split_name}")
        
    except Exception as e:
        print(f"❌ ERRO ao gerar {split_name}:")
        print(f"   {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

# ======== MAIN ASYNC ========
async def main():
    print("\n" + "="*60)
    print("🚀 GERAÇÃO DE DATASET - VALVE WHEEL")
    print("="*60)
    print(f"USD: {USD_PATH}")
    print(f"Output: {OUT_ROOT}")
    print(f"Resolução: {RES[0]}x{RES[1]}")
    print(f"Seed: {SEED}")
    print("="*60)

    # Configura seed
    rep.set_global_seed(SEED)

    # Cria estrutura de pastas
    TRAIN_DIR = os.path.join(OUT_ROOT, "train")
    VAL_DIR = os.path.join(OUT_ROOT, "val")
    os.makedirs(TRAIN_DIR, exist_ok=True)
    os.makedirs(VAL_DIR, exist_ok=True)

    # WARMUP - Inicializa o Replicator
    await warmup_replicator()
    
    print("\n⏸️  Pausa pós-warmup...")
    await asyncio.sleep(3)

    # Gera splits
    print("\n⚙️  Gerando split TRAIN...")
    train_success = await make_split(TRAIN_DIR, N_TRAIN, "TRAIN")
    
    print("\n⏸️  Pausa entre splits (aguardando I/O)...")
    await asyncio.sleep(5)
    
    print("\n⚙️  Gerando split VAL...")
    val_success = await make_split(VAL_DIR, N_VAL, "VAL")
    
    print("\n⏸️  Pausa final (aguardando I/O)...")
    await asyncio.sleep(5)

    # ======== VALIDAÇÃO FINAL ========
    print("\n" + "="*60)
    print("📊 VALIDAÇÃO DE RESULTADOS")
    print("="*60)

    train_ok = validate_split_output(TRAIN_DIR, N_TRAIN, "TRAIN")
    val_ok = validate_split_output(VAL_DIR, N_VAL, "VAL")

    print("\n" + "="*60)
    if train_ok and val_ok:
        print("✅ DATASET GERADO COM SUCESSO!")
        print("="*60)
        print(f"📁 Localização: {OUT_ROOT}")
        print(f"📊 Train: {count_files(TRAIN_DIR, 'rgb_*.png')} imagens RGB")
        print(f"📊 Val: {count_files(VAL_DIR, 'rgb_*.png')} imagens RGB")
    else:
        print("❌ FALHA NA GERAÇÃO DO DATASET")
        print("="*60)
        print("Verifique:")
        print("  1. O arquivo USD existe e está correto")
        print("  2. O objeto tem semantic label 'valve_wheel'")
        print("  3. Os logs acima para mais detalhes")
    print("="*60)

# Executa o script async
asyncio.ensure_future(main())

#!/usr/bin/env python3
"""Análise do log do CuRobo MPC para entender travamentos."""

import re
import matplotlib
matplotlib.use('Agg')  # Backend não interativo
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

LOG_FILE = "/home/spot-teleop/.ros/log/python_331216_1773087354582.log"

def parse_log(filepath):
    """Extrai dados do log do CuRobo MPC."""
    data = defaultdict(list)
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Regex para capturar blocos de step
    step_pattern = re.compile(
        r'=== Step (\d+) \[(OK|BLOCKED)\] ===.*?'
        r'MPC: error=([\d.]+), constraint=([\d.]+), coll_cost=([\d.]+).*?'
        r'TIMING: loop=(\d+)ms, mpc\.step=(\d+)ms, mesh=(\d+)ms, gap=(\d+)ms.*?'
        r'AVG: loop=(\d+)ms, step=(\d+)ms, effective=([\d.]+)Hz.*?'
        r'GPU: util=(\d+)%, mem=(\d+)/(\d+)MB, temp=(\d+)C.*?'
        r'TORCH: alloc=(\d+)MB, reserved=(\d+)MB.*?'
        r'DATA: js_age=(\d+)ms, pub_count=(\d+).*?'
        r'OBST: min_dist=([-\d.]+)m, sphere_idx=(\d+).*?'
        r'CMD: pos=\[([-\d.,]+)\]',
        re.DOTALL
    )
    
    for match in step_pattern.finditer(content):
        data['step'].append(int(match.group(1)))
        data['status'].append(match.group(2))
        data['error'].append(float(match.group(3)))
        data['constraint'].append(float(match.group(4)))
        data['coll_cost'].append(float(match.group(5)))
        data['loop_ms'].append(int(match.group(6)))
        data['mpc_step_ms'].append(int(match.group(7)))
        data['mesh_ms'].append(int(match.group(8)))
        data['gap_ms'].append(int(match.group(9)))
        data['avg_loop_ms'].append(int(match.group(10)))
        data['avg_step_ms'].append(int(match.group(11)))
        data['effective_hz'].append(float(match.group(12)))
        data['gpu_util'].append(int(match.group(13)))
        data['gpu_mem_used'].append(int(match.group(14)))
        data['gpu_mem_total'].append(int(match.group(15)))
        data['gpu_temp'].append(int(match.group(16)))
        data['torch_alloc'].append(int(match.group(17)))
        data['torch_reserved'].append(int(match.group(18)))
        data['js_age_ms'].append(int(match.group(19)))
        data['pub_count'].append(int(match.group(20)))
        data['min_dist'].append(float(match.group(21)))
        data['sphere_idx'].append(int(match.group(22)))
        cmd_pos = [float(x) for x in match.group(23).split(',')]
        data['cmd_pos'].append(cmd_pos)
    
    return data

def analyze_blocked(data):
    """Analisa padrões de BLOCKED."""
    blocked_indices = [i for i, s in enumerate(data['status']) if s == 'BLOCKED']
    ok_indices = [i for i, s in enumerate(data['status']) if s == 'OK']
    
    print("=" * 60)
    print("ANÁLISE DE BLOQUEIOS")
    print("=" * 60)
    print(f"Total de steps: {len(data['step'])}")
    print(f"BLOCKED: {len(blocked_indices)} ({100*len(blocked_indices)/len(data['step']):.1f}%)")
    print(f"OK: {len(ok_indices)} ({100*len(ok_indices)/len(data['step']):.1f}%)")
    
    # Comparar métricas entre BLOCKED e OK
    if blocked_indices and ok_indices:
        print("\n--- Comparação BLOCKED vs OK ---")
        metrics = ['error', 'constraint', 'coll_cost', 'min_dist', 'js_age_ms', 'loop_ms', 'mpc_step_ms']
        for m in metrics:
            blocked_vals = [data[m][i] for i in blocked_indices]
            ok_vals = [data[m][i] for i in ok_indices]
            print(f"{m}:")
            print(f"  BLOCKED: mean={np.mean(blocked_vals):.4f}, max={np.max(blocked_vals):.4f}")
            print(f"  OK:      mean={np.mean(ok_vals):.4f}, max={np.max(ok_vals):.4f}")
    
    # Identificar padrões de quando BLOCKED ocorre
    print("\n--- Padrão de BLOCKED ---")
    if blocked_indices:
        # Ver sequências de BLOCKED
        sequences = []
        current_seq = [blocked_indices[0]]
        for i in range(1, len(blocked_indices)):
            if blocked_indices[i] - blocked_indices[i-1] <= 1:
                current_seq.append(blocked_indices[i])
            else:
                sequences.append(current_seq)
                current_seq = [blocked_indices[i]]
        sequences.append(current_seq)
        
        print(f"Sequências de bloqueio: {len(sequences)}")
        for i, seq in enumerate(sequences[:5]):  # Primeiras 5
            steps = [data['step'][j] for j in seq]
            min_dists = [data['min_dist'][j] for j in seq]
            constraints = [data['constraint'][j] for j in seq]
            print(f"  Seq {i+1}: Steps {steps[0]}-{steps[-1]}, "
                  f"min_dist=[{min(min_dists):.3f}, {max(min_dists):.3f}], "
                  f"constraint_mean={np.mean(constraints):.2f}")

def analyze_timing(data):
    """Analisa timing e delays."""
    print("\n" + "=" * 60)
    print("ANÁLISE DE TIMING")
    print("=" * 60)
    
    # Estatísticas gerais
    print(f"Loop time: mean={np.mean(data['loop_ms']):.1f}ms, "
          f"median={np.median(data['loop_ms']):.1f}ms, "
          f"max={np.max(data['loop_ms'])}ms, "
          f"std={np.std(data['loop_ms']):.1f}ms")
    
    print(f"MPC step: mean={np.mean(data['mpc_step_ms']):.1f}ms, "
          f"median={np.median(data['mpc_step_ms']):.1f}ms, "
          f"max={np.max(data['mpc_step_ms'])}ms")
    
    print(f"Effective Hz: mean={np.mean(data['effective_hz']):.1f}Hz, "
          f"min={np.min(data['effective_hz']):.1f}Hz, "
          f"max={np.max(data['effective_hz']):.1f}Hz")
    
    print(f"JS Age: mean={np.mean(data['js_age_ms']):.1f}ms, "
          f"median={np.median(data['js_age_ms']):.1f}ms, "
          f"max={np.max(data['js_age_ms'])}ms")
    
    # Calcular overhead
    overhead = np.array(data['loop_ms']) - np.array(data['mpc_step_ms'])
    print(f"Overhead (loop - mpc): mean={np.mean(overhead):.1f}ms, max={np.max(overhead)}ms")
    
    # Target rate
    target_rate = 30.0  # Hz do log
    target_loop = 1000 / target_rate
    print(f"\nTarget: {target_rate}Hz ({target_loop:.1f}ms/loop)")
    print(f"% loops > target: {100*np.sum(np.array(data['loop_ms']) > target_loop)/len(data['loop_ms']):.1f}%")

def analyze_collision(data):
    """Analisa comportamento de colisão."""
    print("\n" + "=" * 60)
    print("ANÁLISE DE COLISÃO")
    print("=" * 60)
    
    min_dists = np.array(data['min_dist'])
    
    print(f"Min distance: mean={np.mean(min_dists):.4f}m, "
          f"min={np.min(min_dists):.4f}m, "
          f"max={np.max(min_dists):.4f}m")
    
    # Valores negativos = em colisão
    in_collision = min_dists < 0
    print(f"Steps em colisão (dist < 0): {np.sum(in_collision)} ({100*np.mean(in_collision):.1f}%)")
    
    # Correlação entre distância e bloqueio
    blocked = np.array([1 if s == 'BLOCKED' else 0 for s in data['status']])
    constraint = np.array(data['constraint'])
    
    print(f"\nCorrelação min_dist vs BLOCKED: {np.corrcoef(min_dists, blocked)[0,1]:.3f}")
    print(f"Correlação constraint vs BLOCKED: {np.corrcoef(constraint, blocked)[0,1]:.3f}")
    
    # Análise de constraints > 0 (causa do BLOCKED)
    high_constraint = constraint > 0
    print(f"\nSteps com constraint > 0: {np.sum(high_constraint)} ({100*np.mean(high_constraint):.1f}%)")
    if np.sum(high_constraint) > 0:
        hc_indices = np.where(high_constraint)[0]
        print(f"  Constraint mean: {np.mean(constraint[hc_indices]):.2f}")
        print(f"  Constraint max: {np.max(constraint[hc_indices]):.2f}")

def plot_analysis(data):
    """Gera gráficos de análise."""
    steps = np.array(data['step'])
    
    fig, axes = plt.subplots(4, 2, figsize=(14, 16))
    fig.suptitle('Análise do CuRobo MPC - Log Session', fontsize=14, fontweight='bold')
    
    # Cores para BLOCKED/OK
    colors = ['red' if s == 'BLOCKED' else 'green' for s in data['status']]
    
    # 1. MPC Error e Constraint
    ax1 = axes[0, 0]
    ax1.scatter(steps, data['error'], c=colors, alpha=0.5, s=10, label='Error')
    ax1.set_ylabel('MPC Error', color='blue')
    ax1.set_xlabel('Step')
    ax1.set_title('MPC Error (green=OK, red=BLOCKED)')
    ax1.set_yscale('log')
    ax1.grid(True, alpha=0.3)
    
    ax1b = ax1.twinx()
    constraint_nonzero = [c if c > 0 else np.nan for c in data['constraint']]
    ax1b.scatter(steps, constraint_nonzero, c='orange', alpha=0.7, s=15, marker='x', label='Constraint')
    ax1b.set_ylabel('Constraint (quando > 0)', color='orange')
    
    # 2. Collision Cost e Min Distance
    ax2 = axes[0, 1]
    ax2.plot(steps, data['coll_cost'], 'b-', alpha=0.7, linewidth=0.5)
    ax2.set_ylabel('Collision Cost', color='blue')
    ax2.set_xlabel('Step')
    ax2.set_title('Collision Cost & Min Distance')
    ax2.grid(True, alpha=0.3)
    
    ax2b = ax2.twinx()
    ax2b.plot(steps, data['min_dist'], 'r-', alpha=0.7, linewidth=0.5)
    ax2b.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='Collision boundary')
    ax2b.set_ylabel('Min Distance (m)', color='red')
    
    # 3. Timing
    ax3 = axes[1, 0]
    ax3.plot(steps, data['loop_ms'], 'b-', alpha=0.5, linewidth=0.5, label='Loop')
    ax3.plot(steps, data['mpc_step_ms'], 'g-', alpha=0.5, linewidth=0.5, label='MPC Step')
    ax3.axhline(y=33.3, color='red', linestyle='--', alpha=0.7, label='Target (30Hz)')
    ax3.set_ylabel('Time (ms)')
    ax3.set_xlabel('Step')
    ax3.set_title('Timing (Loop vs MPC Step)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 100)
    
    # 4. Effective Hz
    ax4 = axes[1, 1]
    ax4.scatter(steps, data['effective_hz'], c=colors, alpha=0.5, s=10)
    ax4.axhline(y=30, color='red', linestyle='--', alpha=0.7, label='Target 30Hz')
    ax4.set_ylabel('Effective Hz')
    ax4.set_xlabel('Step')
    ax4.set_title('Effective Frequency (green=OK, red=BLOCKED)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. GPU Utilization e Temperature
    ax5 = axes[2, 0]
    ax5.plot(steps, data['gpu_util'], 'b-', alpha=0.7, linewidth=0.5, label='GPU Util %')
    ax5.set_ylabel('GPU Util %', color='blue')
    ax5.set_xlabel('Step')
    ax5.set_title('GPU Utilization & Temperature')
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(0, 100)
    
    ax5b = ax5.twinx()
    ax5b.plot(steps, data['gpu_temp'], 'r-', alpha=0.7, linewidth=0.5, label='Temp °C')
    ax5b.set_ylabel('Temperature (°C)', color='red')
    ax5b.set_ylim(50, 90)
    
    # 6. JS Age (Joint State Age)
    ax6 = axes[2, 1]
    ax6.scatter(steps, data['js_age_ms'], c=colors, alpha=0.5, s=10)
    ax6.set_ylabel('JS Age (ms)')
    ax6.set_xlabel('Step')
    ax6.set_title('Joint State Age (green=OK, red=BLOCKED)')
    ax6.grid(True, alpha=0.3)
    ax6.set_ylim(0, min(2000, np.max(data['js_age_ms']) * 1.1))
    
    # 7. Sphere Index (qual esfera está mais próxima)
    ax7 = axes[3, 0]
    ax7.scatter(steps, data['sphere_idx'], c=colors, alpha=0.5, s=10)
    ax7.set_ylabel('Sphere Index')
    ax7.set_xlabel('Step')
    ax7.set_title('Closest Collision Sphere (índices baixos = base, altos = gripper)')
    ax7.grid(True, alpha=0.3)
    
    # 8. Histograma de Status por faixa de min_dist
    ax8 = axes[3, 1]
    blocked_dists = [data['min_dist'][i] for i, s in enumerate(data['status']) if s == 'BLOCKED']
    ok_dists = [data['min_dist'][i] for i, s in enumerate(data['status']) if s == 'OK']
    
    bins = np.linspace(min(data['min_dist']), max(data['min_dist']), 30)
    ax8.hist(blocked_dists, bins=bins, alpha=0.7, label='BLOCKED', color='red')
    ax8.hist(ok_dists, bins=bins, alpha=0.5, label='OK', color='green')
    ax8.axvline(x=0, color='black', linestyle='--', alpha=0.7, label='Collision threshold')
    ax8.set_xlabel('Min Distance (m)')
    ax8.set_ylabel('Count')
    ax8.set_title('Distribution of Min Distance by Status')
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/spot-teleop/spot-ros2_ws/curobo_analysis.png', dpi=150)
    print(f"\nGráficos salvos em: /home/spot-teleop/spot-ros2_ws/curobo_analysis.png")
    # plt.show()  # Desabilitado para ambiente headless

def analyze_workspace_limits(data):
    """Analisa se o problema é limite de workspace."""
    print("\n" + "=" * 60)
    print("ANÁLISE DE LIMITES DE WORKSPACE")
    print("=" * 60)
    
    # Analisar posições comandadas
    cmd_pos = np.array(data['cmd_pos'])
    joint_names = ['sh0', 'sh1', 'el0', 'el1', 'wr0', 'wr1']
    
    # Limites típicos do Spot arm (aproximados em radianos)
    limits = {
        'sh0': (-3.14, 3.14),
        'sh1': (-2.79, 0.89),
        'el0': (-3.14, 3.14),
        'el1': (-2.79, 2.79),
        'wr0': (-3.14, 3.14),
        'wr1': (-3.14, 3.14)
    }
    
    print("Posições comandadas (rad):")
    for i, name in enumerate(joint_names):
        vals = cmd_pos[:, i]
        lo, hi = limits[name]
        at_limit = np.sum((vals < lo * 0.95) | (vals > hi * 0.95))
        print(f"  {name}: min={np.min(vals):.3f}, max={np.max(vals):.3f}, "
              f"limit=[{lo:.2f}, {hi:.2f}], near_limit={at_limit}")
    
    # Ver se BLOCKED acontece perto de limites
    blocked_idx = [i for i, s in enumerate(data['status']) if s == 'BLOCKED']
    if blocked_idx:
        blocked_pos = cmd_pos[blocked_idx]
        print("\nPosições durante BLOCKED:")
        for i, name in enumerate(joint_names):
            vals = blocked_pos[:, i]
            print(f"  {name}: range=[{np.min(vals):.3f}, {np.max(vals):.3f}]")

def main():
    print("Carregando log...")
    data = parse_log(LOG_FILE)
    print(f"Parseados {len(data['step'])} steps")
    
    analyze_blocked(data)
    analyze_timing(data)
    analyze_collision(data)
    analyze_workspace_limits(data)
    
    print("\nGerando gráficos...")
    plot_analysis(data)
    
    print("\n" + "=" * 60)
    print("CONCLUSÕES")
    print("=" * 60)
    
    # Resumo automático
    blocked_pct = 100 * sum(1 for s in data['status'] if s == 'BLOCKED') / len(data['status'])
    avg_loop = np.mean(data['loop_ms'])
    avg_constraint_blocked = np.mean([data['constraint'][i] for i, s in enumerate(data['status']) if s == 'BLOCKED'])
    
    print(f"1. Taxa de bloqueio: {blocked_pct:.1f}%")
    print(f"2. Loop médio: {avg_loop:.1f}ms (target: 33.3ms para 30Hz)")
    
    if avg_constraint_blocked > 0:
        print(f"3. BLOCKED causado por constraint={avg_constraint_blocked:.1f} (violação de limites/colisão)")
    
    if np.mean(data['min_dist']) < 0:
        print("4. Braço frequentemente em estado de 'colisão virtual' (min_dist < 0)")
    
    if np.mean(data['js_age_ms']) > 100:
        print(f"5. Joint state delay alto: {np.mean(data['js_age_ms']):.0f}ms médio")

if __name__ == '__main__':
    main()

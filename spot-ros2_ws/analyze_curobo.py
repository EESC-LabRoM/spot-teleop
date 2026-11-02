import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

csv_path = '/home/hipolito/spot-teleop/spot-ros2_ws/curobo_mpc_log_20260424_170756.csv'
df = pd.read_csv(csv_path)

# Filter for control steps where metrics are recorded
df_steps = df[df['event'] == 'control_step'].copy()

# Convert relevant columns to numeric
numeric_cols = ['step', 'pose_error', 'loop_ms', 'mpc_step_ms', 'effective_hz']
for col in numeric_cols:
    df_steps[col] = pd.to_numeric(df_steps[col], errors='coerce')

# Extract roll from curr_pos (index 5 of the pipe-separated string)
# Assuming curr_pos format: val0|val1|val2|val3|val4|val5
def extract_roll(pos_str):
    try:
        if pd.isna(pos_str):
            return np.nan
        parts = pos_str.split('|')
        return float(parts[5])
    except:
        return np.nan

df_steps['curr_roll'] = df_steps['curr_pos'].apply(extract_roll)
df_steps['cmd_roll'] = df_steps['cmd_pos'].apply(extract_roll)

# Drop NaNs
df_steps = df_steps.dropna(subset=['step', 'pose_error', 'curr_roll'])

print("=== STATS TEMPO DE RESPOSTA (NOVO LOG) ===")
print(df_steps[['loop_ms', 'mpc_step_ms', 'effective_hz']].describe())

print("\n=== STATS ERRO (pose_error) ===")
print(df_steps[['pose_error']].describe())

print("\n=== CORRELAÇÃO ===")
corr = df_steps[['pose_error', 'curr_roll']].corr()
print(corr)

# PLOTS
sns.set_theme(style="whitegrid")

# Plot 1: Pose Error & Roll over time (steps)
fig, ax1 = plt.subplots(figsize=(12, 6))

color = 'tab:red'
ax1.set_xlabel('Step')
ax1.set_ylabel('Pose Error', color=color)
ax1.plot(df_steps['step'], df_steps['pose_error'], color=color, label='Pose Error')
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  
color = 'tab:blue'
ax2.set_ylabel('Wrist Roll (arm_wr1) [rad]', color=color)  
ax2.plot(df_steps['step'], df_steps['curr_roll'], color=color, label='Wrist Roll', alpha=0.7)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  
plt.title('Pose Error vs Wrist Roll ao longo dos Steps')
plt.savefig('/home/hipolito/spot-teleop/spot-ros2_ws/plot_error_vs_roll_novo.png')
plt.close()

# Scatter plot: Error vs Roll
plt.figure(figsize=(8, 6))
sns.scatterplot(data=df_steps, x='curr_roll', y='pose_error', alpha=0.5)
plt.title('Correlação: Pose Error vs Wrist Roll')
plt.xlabel('Wrist Roll (rad)')
plt.ylabel('Pose Error')
plt.tight_layout()
plt.savefig('/home/hipolito/spot-teleop/spot-ros2_ws/plot_scatter_error_roll.png')
plt.close()

print("\nPlots gerados com sucesso.")

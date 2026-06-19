import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==============================================================================
# 1. CONFIGURATION (Change TRACK_NAME to plot different circuits)
# ==============================================================================
TRACK_NAME = "Monza"  # Change this to "Austin", "Silverstone", etc.

DATA_DIR = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\racetrack-database\racelines"
OUTPUT_DIR = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\racetrack-database\Output"
F1_TRACK_WIDTH = 12.0  # Must match the width used in the optimization script

# File paths
original_csv_path = os.path.join(DATA_DIR, f"{TRACK_NAME}.csv")
optimized_csv_path = os.path.join(OUTPUT_DIR, f"{TRACK_NAME}_optimized.csv")

# ==============================================================================
# 2. LOAD DATA
# ==============================================================================
if not os.path.exists(original_csv_path):
    print(f"❌ Original track file not found: {original_csv_path}")
    exit()

if not os.path.exists(optimized_csv_path):
    print(f"❌ Optimized track file not found: {optimized_csv_path}\nRun the optimizer script first!")
    exit()

print(f"📂 Loading Original Centerline: {TRACK_NAME}.csv")
df_orig = pd.read_csv(original_csv_path)

# Handle different column namings dynamically
x_col = [col for col in df_orig.columns if 'x' in col.lower()][0]
y_col = [col for col in df_orig.columns if 'y' in col.lower()][0]
orig_x, orig_y = df_orig[x_col].values, df_orig[y_col].values

print(f"📂 Loading Optimized Line: {TRACK_NAME}_optimized.csv")
df_opt = pd.read_csv(optimized_csv_path)
opt_x, opt_y = df_opt['x'].values, df_opt['y'].values

# ==============================================================================
# 3. RECONSTRUCT TRACK BOUNDARIES
# ==============================================================================
# Calculate the normal vectors to draw the track walls perfectly
dx = np.gradient(orig_x)
dy = np.gradient(orig_y)
ds = np.sqrt(dx**2 + dy**2) + 1e-6
nx, ny = -dy / ds, dx / ds 

track_half = F1_TRACK_WIDTH / 2.0
left_bound_x = orig_x + nx * track_half
left_bound_y = orig_y + ny * track_half
right_bound_x = orig_x - nx * track_half
right_bound_y = orig_y - ny * track_half

# ==============================================================================
# 4. PLOTTING
# ==============================================================================
plt.figure(figsize=(14, 10))
plt.gcf().canvas.manager.set_window_title(f'{TRACK_NAME} - Trajectory Comparison')

# Plot Track Limits
plt.plot(left_bound_x, left_bound_y, color='black', linewidth=2, label='Track Limits')
plt.plot(right_bound_x, right_bound_y, color='black', linewidth=2)

# Plot Original Centerline
plt.plot(orig_x, orig_y, color='gray', linestyle='--', linewidth=1.5, alpha=0.7, label='Original Centerline')

# Plot Optimized Racing Line
plt.plot(opt_x, opt_y, color='red', linestyle='-', linewidth=2.5, label='Optimized Alpha Racing Line')

# Formatting
plt.axis("equal")
plt.title(f"Path Planning Comparison: {TRACK_NAME} Circuit\nNotice how the Red line clips the apexes compared to the Gray centerline", fontsize=14)
plt.xlabel("X Position (meters)", fontsize=12)
plt.ylabel("Y Position (meters)", fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)

# Make legend readable
plt.legend(loc="upper right", framealpha=0.9, fontsize=11)

print("✅ Plot generated successfully. Close the window to exit.")
plt.show()
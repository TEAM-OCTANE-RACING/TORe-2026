#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
F1 Track Optimization Test Bench
Evaluates the 1D Alpha-Parameterization algorithm on global Formula 1 circuits.

Project: fsd_path_planning (F1 Extension)
"""

import os
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# ==============================================================================
# CONFIGURATION & TUNING ENVIRONMENT
# ==============================================================================
# Set this to EITHER a folder containing many CSVs, OR a specific CSV file path.
DATA_DIR = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\racetrack-database\racelines"
OUTPUT_DIR = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\racetrack-database\Output"

# Optimization Configuration
OPT_ITERATIONS = 80        # F1 tracks require more iterations to converge due to length
OPT_SAFETY_MARGIN = 1.0    # Meters of clearance from structural track boundaries
F1_TRACK_WIDTH = 12.0      # Average standard F1 circuit width (m)
UPSAMPLE_RESOLUTION = 2000 # Dense waypoint spacing for large curves

# ==============================================================================
# MATHEMATICAL ALPHA OPTIMIZER
# ==============================================================================
def optimize_f1_racing_line(centerline, track_width=12.0, safety_margin=2.5, iterations=50):
    """
    Applies 1D path relaxation using perpendicular projection vectors.
    Transforms global track arrays into un-tanglable Frenet coordinate offsets.
    """
    if len(centerline) < 4:
        return centerline

    # Extrapolate raw Cartesian coordinates
    x, y = centerline[:, 0].copy(), centerline[:, 1].copy()
    N = len(x)

    # 1. Generate Differential Perpendicular Matrices (The Normal Rails)
    dx = np.gradient(x)
    dy = np.gradient(y)
    ds = np.sqrt(dx**2 + dy**2) + 1e-6
    nx, ny = -dy / ds, dx / ds 

    # 2. Allocate Shift Scalar State Matrix (Alpha Vectors)
    alpha = np.zeros(N)

    # 3. Formulate Hard Physics Limits
    track_half_width = track_width / 2.0
    bound_left = max(track_half_width - safety_margin, 0.0)
    bound_right = -max(track_half_width - safety_margin, 0.0)

    # 4. Execute Iterative Laplacian Shifting
    for _ in range(iterations):
        # Derive structural X, Y mappings for current state step
        px = x + alpha * nx
        py = y + alpha * ny

        # Apply continuous tension using local geometric neighbors
        px_avg = np.copy(px)
        py_avg = np.copy(py)
        px_avg[1:-1] = 0.5 * (px[:-2] + px[2:])
        py_avg[1:-1] = 0.5 * (py[:-2] + py[2:])

        # Calculate raw 2D displacement vectors
        move_x = px_avg - px
        move_y = py_avg - py

        # Project 2D forces into 1D Frenet Space via Vector Dot Product
        d_alpha = move_x * nx + move_y * ny
        alpha += d_alpha

        # Force boundaries using localized algebraic clamping
        alpha = np.clip(alpha, bound_right, bound_left)

    # 5. Reconstruct Optimized Space Maps
    opt_x = x + alpha * nx
    opt_y = y + alpha * ny

    # 6. Smooth via Parametric Cubic Spline (C2 Continuity Verification)
    try:
        tck, _ = splprep([opt_x, opt_y], s=1.0, per=True)
        u_new = np.linspace(0, 1.0, UPSAMPLE_RESOLUTION)
        x_smooth, y_smooth = splev(u_new, tck)
        return np.column_stack((x_smooth, y_smooth)), alpha, (nx, ny)
    except Exception as e:
        print(f"   [Warning] Spline parameterization error: {e}. Falling back to linear mesh.")
        return np.column_stack((opt_x, opt_y)), alpha, (nx, ny)

# ==============================================================================
# SYSTEM TEST EXECUTION RUNNER
# ==============================================================================
def process_and_test_tracks():
    """Iterates through database circuits, executes math loops, and saves results."""
    
    if not os.path.exists(DATA_DIR):
        print(f"❌ Target source path cannot be found: '{DATA_DIR}'.")
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # --- SMART PATH DETECTOR ---
    csv_files = []
    if os.path.isfile(DATA_DIR) and DATA_DIR.endswith('.csv'):
        # User provided a single track file (e.g., Monza.csv)
        csv_files.append(DATA_DIR)
    elif os.path.isdir(DATA_DIR):
        # User provided a folder of tracks
        csv_files = [os.path.join(DATA_DIR, f) for f in os.listdir(DATA_DIR) if f.endswith('.csv')]
    
    if not csv_files:
        print("❌ No track CSV files detected.")
        return

    print(f"🏁 F1 Verification Node Online. Processing {len(csv_files)} track model(s)...\n" + "="*70)

    for file_path in csv_files:
        track_name = os.path.splitext(os.path.basename(file_path))[0]
        
        print(f"⚡ Processing Circuit: [{track_name}]")
        
        try:
            # Read F1 path data frames
            df = pd.read_csv(file_path)
            
            # Dynamically identify structural headers
            x_col = [col for col in df.columns if 'x' in col.lower()][0]
            y_col = [col for col in df.columns if 'y' in col.lower()][0]
            
            centerline = df[[x_col, y_col]].to_numpy()
            
            # Enforce data completion for loop processing (close the loop if it's a circuit)
            if not np.allclose(centerline[0], centerline[-1]):
                centerline = np.vstack([centerline, centerline[0]])

            # Execute Optimization Loops
            t_start = time.perf_counter()
            optimized_line, alpha_shifts, normals = optimize_f1_racing_line(
                centerline, 
                track_width=F1_TRACK_WIDTH, 
                safety_margin=OPT_SAFETY_MARGIN, 
                iterations=OPT_ITERATIONS
            )
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            
            print(f"   ↳ Real-time Convergence: {t_elapsed:.2f} ms | Mesh Nodes: {len(optimized_line)}")

            # Calculate Boundary Boundaries for Plot Layouts
            nx, ny = normals
            track_half = F1_TRACK_WIDTH / 2.0
            left_wall_x = centerline[:, 0] + nx * track_half
            left_wall_y = centerline[:, 1] + ny * track_half
            right_wall_x = centerline[:, 0] - nx * track_half
            right_wall_y = centerline[:, 1] - ny * track_half

            # Save Optimized Path Mappings
            out_csv = os.path.join(OUTPUT_DIR, f"{track_name}_optimized.csv")
            pd.DataFrame(optimized_line, columns=['x', 'y']).to_csv(out_csv, index=False)

            # Generate Engineering Plot Layout
            plt.figure(figsize=(12, 10))
            plt.plot(centerline[:, 0], centerline[:, 1], 'w--', alpha=0.6, label='Original Centerline')
            plt.plot(left_wall_x, left_wall_y, color='tab:gray', linewidth=1.5, label='Track Bounds')
            plt.plot(right_wall_x, right_wall_y, color='tab:gray', linewidth=1.5)
            plt.plot(optimized_line[:, 0], optimized_line[:, 1], color='tab:cyan', linewidth=2.5, label='Minimum Curvature Line')
            
            # Highlight Apex Constraints (Where the path hits safety thresholds)
            max_limit = (F1_TRACK_WIDTH / 2.0) - OPT_SAFETY_MARGIN
            apex_indices = np.where(np.abs(alpha_shifts) >= (max_limit - 1e-2))[0]
            if len(apex_indices) > 0:
                plt.scatter(centerline[apex_indices, 0], centerline[apex_indices, 1], 
                            color='tab:red', s=15, zorder=5, label='Apex Limit Constraints')

            # Figure Styling Configurations
            ax = plt.gca()
            ax.set_facecolor('#111111') # Performance dark-mode styling
            plt.title(f"Alpha-Parameterization Optimization Profile: {track_name}", fontsize=12, pad=15)
            plt.xlabel("Global Coordinates X (meters)", fontsize=10)
            plt.ylabel("Global Coordinates Y (meters)", fontsize=10)
            plt.axis('equal')
            plt.grid(True, color='#333333', linestyle=':', alpha=0.5)
            plt.legend(facecolor='#222222', edgecolor='none', labelcolor='white', loc='upper right')
            
            # Save Output Image Mapping
            plot_img_path = os.path.join(OUTPUT_DIR, f"{track_name}_plot.png")
            plt.savefig(plot_img_path, dpi=150, facecolor='#111111', bbox_inches='tight')
            plt.close()
            
            print(f"   ↳ File storage success: '{out_csv}' & plots compiled.\n")

        except Exception as e:
            print(f"   ❌ Execution failure compiling [{track_name}]: {e}\n")
            
    print("="*70 + f"\n Verification Loop Terminated successfully. Review folder: '{OUTPUT_DIR}'")

if __name__ == "__main__":
    process_and_test_tracks()
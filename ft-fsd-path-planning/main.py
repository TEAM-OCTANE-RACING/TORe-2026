import json
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
import time

from fsd_path_planning.full_pipeline.full_pipeline import PathPlanner
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes

# 🚀 IMPORT THE NEW UNIFIED TRAJECTORY OPTIMIZER LIBRARY
from fsd_path_planning.utils.trajectory_optimization import TrajectoryOptimizer

# ==========================================================
# 0. CONFIGURATION & TUNING PARAMETERS
# ==========================================================
TOTAL_LAPS = 2
dt = 0.05
velocity = 3.0

# --- CAR DIMENSIONS & LIMITS ---
TRACK_WIDTH = 1.325
CAR_HALF_WIDTH = TRACK_WIDTH / 2.0  # 0.6625m

vehicle_limits = {
    'ay_max': 1.27 * 9.81,
    'ax_max': 0.64 * 9.81,
    'adec_max': 1.2 * 9.81,
    'v_max': 21.427
}

# --- TRACKING PARAMS (Perfect Tracking) ---
LOOKAHEAD_LAP1 = 2.0 
LOOKAHEAD_LAP2 = 1.0 

# --- OPTIMIZATION SETTINGS ---
OPT_ITERATIONS = 200    
OPT_SAFETY_MARGIN = 0.1 
DETECTION_RADIUS = 3.0
MIN_LAP_DISTANCE = 150.0

# ==========================================================
# 1. HELPER FUNCTIONS
# ==========================================================
def get_target_from_global_path(path, current_pos, lookahead):
    dists = np.linalg.norm(path[:, :2] - current_pos, axis=1) # Only compare X, Y
    closest_idx = np.argmin(dists)
    
    for i in range(closest_idx, closest_idx + 100):
        idx = i % len(path)
        if np.linalg.norm(path[idx, :2] - current_pos) >= lookahead:
            return path[idx, :2]
    return path[(closest_idx + 10) % len(path), :2]


# ==========================================================
# 2. LOAD TRACK & CONES
# ==========================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Point directly to your layout-merchant folder!
TRACK_FILE = r"C:\Users\Ayush\layout-merchant\layouts\track_7.json" 

OUTPUT_DIR = os.path.join(BASE_DIR, "fsd_path_planning", "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_DIR = os.path.join(BASE_DIR, "fsd_path_planning", "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

with open(TRACK_FILE, "r") as f:
    track = json.load(f)

x_track, y_track, color = np.array(track["x"]), np.array(track["y"]), np.array(track["color"])

LEFT_COLOR, RIGHT_COLOR, ORANGE_COLOR = 2, 1, 4 
left_cones = np.column_stack([x_track[color == LEFT_COLOR], y_track[color == LEFT_COLOR]])
right_cones = np.column_stack([x_track[color == RIGHT_COLOR], y_track[color == RIGHT_COLOR]])
orange_cones = np.column_stack([x_track[color == ORANGE_COLOR], y_track[color == ORANGE_COLOR]])

cones = [np.zeros((0, 2)) for _ in ConeTypes]
cones[ConeTypes.LEFT], cones[ConeTypes.RIGHT] = left_cones, right_cones
if hasattr(ConeTypes, 'ORANGE_BIG'):
    cones[ConeTypes.ORANGE_BIG] = orange_cones

# ==========================================================
# 3. INITIALIZE SIMULATION STATE
# ==========================================================
vehicle_position = np.array(track["start_position"], dtype=float)
yaw = np.deg2rad(track["start_orientation"])
vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])

planner = PathPlanner(MissionTypes.trackdrive)

# 🚀 INITIALIZE OUR NEW LIBRARY
optimizer = TrajectoryOptimizer(vehicle_limits) 

frames, trajectory_log, lap1_trajectory = [], [], []
optimized_racing_line = None

current_lap = 1
lap_distance_travelled = 0.0
step = 0

print(f"🏁 Starting Planning Simulation: {TOTAL_LAPS} Laps.")

# ==========================================================
# 4. CLOSED-LOOP SIMULATION
# ==========================================================
while current_lap <= TOTAL_LAPS:
    
    if frames: 
        lap_distance_travelled += np.linalg.norm(vehicle_position - frames[-1]["pos"])

    # --- PATH PLANNING ---
    if current_lap == 1:
        path = planner.calculate_path_in_global_frame(cones, vehicle_position, vehicle_direction)
        if path is None or path.shape[0] < 2:
            print("❌ Planner failed, stopping simulation")
            break
        
        target = path[-1, 1:3] 
        for pt in path[:, 1:3]:
            if np.linalg.norm(pt - vehicle_position) >= LOOKAHEAD_LAP1:
                target = pt
                break
                
        lap1_trajectory.append([vehicle_position[0], vehicle_position[1]])
    else:
        target = get_target_from_global_path(optimized_racing_line, vehicle_position, LOOKAHEAD_LAP2)
        path = optimized_racing_line[:, :2] # Only pass X, Y for plotting

    trajectory_log.append([step, vehicle_position[0], vehicle_position[1]])

    frames.append({
        "lap": current_lap, "pos": vehicle_position.copy(),
        "path": path, "target": target
    })

    # --- PERFECT TRACKING (NO CONTROLLER) ---
    direction_vec = target - vehicle_position
    dist_to_target = np.linalg.norm(direction_vec)
    
    if dist_to_target > 1e-6:
        direction_vec /= dist_to_target
        vehicle_direction = direction_vec
        vehicle_position += velocity * dt * direction_vec

    # --- LAP COMPLETION CHECK ---
    min_dist_to_orange = np.min(np.linalg.norm(orange_cones - vehicle_position, axis=1)) if len(orange_cones) > 0 else float('inf')

    if min_dist_to_orange < DETECTION_RADIUS and lap_distance_travelled > MIN_LAP_DISTANCE:
        print(f"\n✅ Lap {current_lap} Completed!")
        
        if current_lap == 1:
            print(f"⚙️ Running Unified Trajectory Optimization Pipeline...")
            centerline_lap1 = np.array(lap1_trajectory)
            
            # 🚀 RUN THE ENTIRE MATH PIPELINE IN ONE LINE!
            results = optimizer.generate_full_trajectory(
                centerline=centerline_lap1, 
                track_half_width=1.5, 
                safety_margin=OPT_SAFETY_MARGIN, 
                iterations=OPT_ITERATIONS,
                standing_start=True
            )
            
            optimized_racing_line = results["trajectory_array"]
            optimal_time = results["lap_time"]
            
            print("\n" + "="*50)
            print("⏱️  LAP TIME PREDICTION")
            print("="*50)
            print(f"Predicted Optimal Lap Time:  {optimal_time:.2f} seconds 🔥")
            print("="*50 + "\n")

            # --- FILE SAVING BLOCK ---
            timestamp = int(time.time())
            
            # 1. SAVE CENTERLINE (LAP 1)
            csv_centerline_path = os.path.join(OUTPUT_DIR, f"track_7_centerline_lap1.csv")
            with open(csv_centerline_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(centerline_lap1) 
            print(f"💾 Lap 1 Centerline saved to: {csv_centerline_path}")

            # 2. SAVE OPTIMAL TRAJECTORY (LAP 2)
            csv_optimal_path = os.path.join(OUTPUT_DIR, f"track_7_optimal_lap2.csv")
            with open(csv_optimal_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "heading", "curvature", "velocity"])
                writer.writerows(optimized_racing_line) 
            print(f"💾 Lap 2 Trajectory saved to: {csv_optimal_path}")

        current_lap += 1
        lap_distance_travelled = 0.0 

    step += 1

print("🏁 Simulation Ended.")

# ==========================================================
# 5. VISUALIZATION
# ==========================================================
traj_array = np.array(trajectory_log)

def play_lap_animation(lap_number, lap_frames):
    plt.figure(figsize=(12, 10))
    plt.gcf().canvas.manager.set_window_title(f'Lap {lap_number} Simulation')
    
    stop_animation = False
    def on_key(event):
        nonlocal stop_animation
        if event.key == "q":
            stop_animation = True
            plt.close()
    plt.gcf().canvas.mpl_connect("key_press_event", on_key)

    print(f"Playing Animation for Lap {lap_number}... (Close window to proceed)")

    for i, frame in enumerate(lap_frames):
        if stop_animation: break
        plt.clf()

        plt.scatter(left_cones[:, 0], left_cones[:, 1], c="blue", s=20, zorder=5, label="Left Cones")
        plt.scatter(right_cones[:, 0], right_cones[:, 1], c="gold", s=20, zorder=5, label="Right Cones")
        
        # Plot trajectory history up to this frame
        history = np.array([f["pos"] for f in lap_frames[:i+1]])
        if len(history) > 0:
            plt.plot(history[:, 0], history[:, 1], 'r--', linewidth=2, label="Vehicle Trajectory")

        plt.legend()
        plt.title(f"Lap {lap_number} Simulation (Press 'q' to skip/close)")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True)
        plt.pause(0.01)

# Play Laps
lap1_frames = [f for f in frames if f["lap"] == 1]
if lap1_frames: play_lap_animation(1, lap1_frames); plt.show()

lap2_frames = [f for f in frames if f["lap"] == 2]
if lap2_frames: play_lap_animation(2, lap2_frames); plt.show()
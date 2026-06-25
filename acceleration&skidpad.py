import json
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
import time

from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes
from fsd_path_planning.utils.trajectory_optimization import TrajectoryOptimizer

# ==========================================================
# 1. MISSION CONTROL SETTINGS (STATIC ONLY)
# ==========================================================
# 🚀 CHANGE THIS TO TEST EITHER SKIDPAD OR ACCELERATION
CURRENT_MISSION = MissionTypes.acceleration  # Options: skidpad, acceleration

if CURRENT_MISSION == MissionTypes.skidpad:
    TRACK_FILE_NAME = "skidpad.json"
elif CURRENT_MISSION == MissionTypes.acceleration:
    TRACK_FILE_NAME = "acceleration.json"
else:
    raise ValueError("❌ This script is only for Static Missions: Skidpad or Acceleration.")

# Physics & Tuning Settings
dt = 0.05
DETECTION_RADIUS = 3.0    

vehicle_limits = {
    'ay_max': 1.27 * 9.81, 'ax_max': 0.64 * 9.81, 
    'adec_max': 1.2 * 9.81, 'v_max': 21.427
}

# ==========================================================
# 2. HELPER FUNCTIONS (MATHEMATICAL PATHS)
# ==========================================================
def generate_skidpad_centerline():
    """Generates a mathematically perfect FSG compliant Skidpad path locked to absolute (0,0)."""
    R = 9.125 
    y_entry = np.linspace(-20, 0, 50)
    x_entry = np.zeros_like(y_entry)
    
    t_R = np.linspace(np.pi, -3*np.pi, 400) 
    x_R = R + R * np.cos(t_R)
    y_R = R * np.sin(t_R)
    
    t_L = np.linspace(0, 4*np.pi, 400) 
    x_L = -R + R * np.cos(t_L)
    y_L = R * np.sin(t_L)
    
    y_exit = np.linspace(0, 25, 50)
    x_exit = np.zeros_like(y_exit)
    
    x_base = np.concatenate([x_entry, x_R[1:], x_L[1:], x_exit[1:]])
    y_base = np.concatenate([y_entry, y_R[1:], y_L[1:], y_exit[1:]])
    
    return np.column_stack([x_base, y_base])

def generate_accel_path(start_pos, start_yaw):
    """Generates an 85m straight line."""
    direction = np.array([np.cos(start_yaw), np.sin(start_yaw)])
    distances = np.linspace(0, 85, 100) 
    return start_pos + distances[:, None] * direction

def get_target_from_fixed_path(path, current_pos, lookahead, last_idx):
    """Strictly sequential tracker to prevent jumping across Figure-8 intersections."""
    search_window = min(50, len(path) - last_idx)
    if search_window == 0: return path[-1], last_idx
    
    dists = np.linalg.norm(path[last_idx:last_idx+search_window] - current_pos, axis=1)
    closest_idx = last_idx + np.argmin(dists)
    
    for i in range(closest_idx, len(path)):
        if np.linalg.norm(path[i] - current_pos) >= lookahead:
            return path[i], closest_idx
            
    return path[-1], closest_idx

# ==========================================================
# 3. LOAD TRACK & INITIALIZE
# ==========================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 💡 Ensure layouts folder path is correct for your system!
TRACK_FILE = os.path.join(BASE_DIR, "layouts", TRACK_FILE_NAME) 

if not os.path.exists(TRACK_FILE):
    raise FileNotFoundError(f"❌ ERROR: The layout file '{TRACK_FILE_NAME}' is missing from the 'layouts' folder.")

OUTPUT_DIR = os.path.join(BASE_DIR, "fsd_path_planning", "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

with open(TRACK_FILE, "r") as f: track = json.load(f)

x_track, y_track, color = np.array(track["x"]), np.array(track["y"]), np.array(track["color"])
left_cones = np.column_stack([x_track[color == 2], y_track[color == 2]])
right_cones = np.column_stack([x_track[color == 1], y_track[color == 1]])
orange_cones = np.column_stack([x_track[color == 4], y_track[color == 4]])

vehicle_position = np.array(track["start_position"], dtype=float)
yaw = np.deg2rad(track["start_orientation"])

optimizer = TrajectoryOptimizer(vehicle_limits) 
frames = []
tracking_idx = 0 

print(f"🏁 Mission: {CURRENT_MISSION.name.upper()} | Map: {TRACK_FILE_NAME}")

# ==========================================================
# 4. PRE-COMPUTE STATIC MISSIONS (MATH + OPTIMIZER)
# ==========================================================
print("⚙️ Generating Pre-calculated Mathematical Track Geometry...")
if CURRENT_MISSION == MissionTypes.skidpad:
    fixed_centerline = generate_skidpad_centerline() 
else:
    fixed_centerline = generate_accel_path(vehicle_position, yaw)
    
x_opt, y_opt = fixed_centerline[:, 0], fixed_centerline[:, 1]
dx, dy = np.gradient(x_opt), np.gradient(y_opt)
opt_heading = np.arctan2(dy, dx)
ddx, ddy = np.gradient(dx), np.gradient(dy)
opt_curvature = (dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)

s_array = np.zeros(len(x_opt))
s_array[1:] = np.cumsum(np.sqrt(np.diff(x_opt)**2 + np.diff(y_opt)**2))

print("⚙️ Calculating Optimal Velocity Profile...")
opt_velocity = optimizer.calculate_velocity_profile(s_array, opt_curvature, standing_start=True)
optimal_time = optimizer.calculate_lap_time(s_array, opt_velocity)

optimized_racing_line = np.column_stack((x_opt, y_opt, opt_heading, opt_curvature, opt_velocity))

# ==========================================================
# 5. TIME INTEGRATION (NEW TELEMETRY FORMAT)
# ==========================================================
print(f"⚙️ Generating Time-Synchronized Optimal Trajectory...")

N = len(opt_velocity)
t_array = np.zeros(N)

dx_diff = np.diff(x_opt)
dy_diff = np.diff(y_opt)
ds = np.sqrt(dx_diff**2 + dy_diff**2)

# Average velocity between two points
v_avg = (opt_velocity[:-1] + opt_velocity[1:]) / 2.0
v_avg[v_avg < 0.5] = 0.5 # Prevent dividing by zero

dt_array = ds / v_avg
t_array[1:] = np.cumsum(dt_array) # Cumulative time sum

# Combine everything into a 6-column matrix 
time_synced_trajectory = np.column_stack((t_array, x_opt, y_opt, opt_heading, opt_curvature, opt_velocity))

print("\n" + "="*50)
print(f"⏱️  {CURRENT_MISSION.name.upper()} PREDICTION")
print("="*50)
print(f"Theoretical Target Event Time:  {optimal_time:.2f} seconds 🔥")
print("="*50 + "\n")

# --- FILE SAVING BLOCK ---
timestamp = int(time.time())
csv_path = os.path.join(OUTPUT_DIR, f"{CURRENT_MISSION.name}_telemetry_{timestamp}.csv")

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_sec", "x", "y", "heading", "curvature", "target_velocity"])
    writer.writerows(time_synced_trajectory)

print(f"💾 Time-synced trajectory saved to: {csv_path}")

# ==========================================================
# 6. UNIFIED DYNAMIC SIMULATION LOOP (VISUALIZATION DRIVER)
# ==========================================================
# Now we just drive the pre-calculated line perfectly for the animation!
step = 0
LOOKAHEAD_LAP = 1.5

while True:
    
    # ----------------------------------------------------
    # A. TRACKING
    # ----------------------------------------------------
    target, tracking_idx = get_target_from_fixed_path(optimized_racing_line[:, :2], vehicle_position, LOOKAHEAD_LAP, tracking_idx)
    path = optimized_racing_line[:, :2] 

    frames.append({"pos": vehicle_position.copy(), "path": path})

    # ----------------------------------------------------
    # B. KINEMATICS (VEHICLE MOVEMENT)
    # ----------------------------------------------------
    direction_vec = target - vehicle_position
    dist_to_target = np.linalg.norm(direction_vec)
    
    if dist_to_target > 1e-6:
        direction_vec /= dist_to_target
        sim_velocity = optimized_racing_line[min(tracking_idx, len(optimized_racing_line)-1), 4]
        sim_velocity = max(sim_velocity, 1.0)
        vehicle_position += sim_velocity * dt * direction_vec

    # ----------------------------------------------------
    # C. COMPLETION CHECK
    # ----------------------------------------------------
    if tracking_idx >= len(optimized_racing_line) - 5:
        print(f"\n✅ Simulation Completed Successfully!")
        break
        
    step += 1

print("🏁 Finished.")

# ==========================================================
# 7. VISUALIZATION
# ==========================================================
def play_lap_animation(lap_frames):
    plt.figure(figsize=(12, 10))
    plt.gcf().canvas.manager.set_window_title(f'{CURRENT_MISSION.name.upper()} - Run Visualization')
    
    stop_animation = False
    def on_key(event):
        nonlocal stop_animation
        if event.key == "q": stop_animation = True; plt.close()
    plt.gcf().canvas.mpl_connect("key_press_event", on_key)

    # Skip frames to speed up animation playback
    for i in range(0, len(lap_frames), 2): 
        if stop_animation: break                                                                 
        plt.clf()

        plt.scatter(left_cones[:, 0], left_cones[:, 1], c="blue", s=20, zorder=5, label="Left Cones")
        plt.scatter(right_cones[:, 0], right_cones[:, 1], c="gold", s=20, zorder=5, label="Right Cones")
        if len(orange_cones) > 0:
            plt.scatter(orange_cones[:, 0], orange_cones[:, 1], c="darkorange", s=50, marker="^", zorder=6, label="Orange Cones")
            
        history = np.array([f["pos"] for f in lap_frames[:i+1]])
        if len(history) > 0:
            plt.plot(history[:, 0], history[:, 1], 'r--', linewidth=2.5, label="Vehicle Trajectory")

        plt.legend()
        plt.title(f"{CURRENT_MISSION.name.upper()} Simulation (Press 'q' to finish)")
        plt.axis('equal')
        plt.grid(True)
        plt.pause(0.001)

play_lap_animation(frames)
plt.show()
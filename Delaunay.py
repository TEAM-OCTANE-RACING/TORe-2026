import json
import csv
import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib.collections import LineCollection

from fsd_path_planning.full_pipeline.full_pipeline import PathPlanner
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes


# ==========================================================
# 1. Load track JSON
# ==========================================================
with open(r"C:\Users\Ayush\layout-merchant\layouts\track_3.json", "r") as f:
    track = json.load(f)

x = np.array(track["x"])
y = np.array(track["y"])
color = np.array(track["color"])


# ==========================================================
# 2. Extract cones (ADDED ORANGE CONES)
# ==========================================================
LEFT_COLOR = 2
RIGHT_COLOR = 1
ORANGE_COLOR = 4  # Assuming 4 is used for orange cones based on the JSON data

left_cones = np.column_stack([x[color == LEFT_COLOR], y[color == LEFT_COLOR]])
right_cones = np.column_stack([x[color == RIGHT_COLOR], y[color == RIGHT_COLOR]])
orange_cones = np.column_stack([x[color == ORANGE_COLOR], y[color == ORANGE_COLOR]])

cones = [np.zeros((0, 2)) for _ in ConeTypes]
cones[ConeTypes.LEFT] = left_cones
cones[ConeTypes.RIGHT] = right_cones

# If your specific version of fsd_path_planning uses orange cones for planning, you can assign them here.
# Otherwise, we just use the `orange_cones` array for our stopping logic below.
if hasattr(ConeTypes, 'ORANGE_BIG'):
    cones[ConeTypes.ORANGE_BIG] = orange_cones


# ==========================================================
# 3. Initial vehicle pose
# ==========================================================
vehicle_position = np.array(track["start_position"], dtype=float)
start_position = vehicle_position.copy()

yaw = np.deg2rad(track["start_orientation"])
vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])


# ==========================================================
# 4. Simulation parameters
# ==========================================================
dt = 0.1
velocity = 7.0
LOOKAHEAD_DISTANCE = 25.0

planner = PathPlanner(MissionTypes.trackdrive)


# ==========================================================
# 5. Helper: truncate planned path
# ==========================================================
def truncate_path_by_distance(path, max_distance):
    out = [path[0]]
    acc = 0.0

    for i in range(1, len(path)):
        d = np.linalg.norm(path[i, 1:3] - path[i - 1, 1:3])
        acc += d
        if acc > max_distance:
            break
        out.append(path[i])

    return np.array(out)


# ==========================================================
# 6. Logs
# ==========================================================
frames = [] 
trajectory_log = []  # step, x, y


# ==========================================================
# 7. Track-end stopping variables
# ==========================================================
distance_travelled = 0.0
DETECTION_RADIUS = 3.0       # meters to trigger the orange cone stop
MIN_LAP_DISTANCE = 150.0     # Ensures it doesn't stop immediately at the start line


# ==========================================================
# 8. Closed-loop simulation (STOP WHEN ORANGE CONES APPEAR)
# ==========================================================
step = 0

while True:
    if frames:
        distance_travelled += np.linalg.norm(vehicle_position - frames[-1]["pos"])

    # --- PLAN ---
    path = planner.calculate_path_in_global_frame(
        cones,
        vehicle_position,
        vehicle_direction
    )

    if path is None or path.shape[0] < 2:
        print("❌ Planner failed, stopping simulation")
        break

    # --- Apply lookahead distance ---
    path = truncate_path_by_distance(path, LOOKAHEAD_DISTANCE)

    # --- Log trajectory ---
    trajectory_log.append([step, vehicle_position[0], vehicle_position[1]])

    # Save everything for this frame (including the FILTERED valid edges)
    frames.append({
        "pos": vehicle_position.copy(),
        "dir": vehicle_direction.copy(),
        "path": path,
        "valid_edges": getattr(planner.pathing, "valid_edges", []),
        "waypoints": getattr(planner.pathing, "last_midpoints", None)
    })

    # --- Follow path ---
    target = path[1, 1:3]
    direction_vec = target - vehicle_position

    if np.linalg.norm(direction_vec) < 1e-6:
        print("❌ Vehicle stalled, stopping")
        break

    direction_vec /= np.linalg.norm(direction_vec)

    # --- Update vehicle position (constant speed) ---
    vehicle_position += velocity * dt * direction_vec
    vehicle_direction = direction_vec

    # ======================================================
    #  NEW STOP CONDITION: ORANGE CONES DETECTED
    # ======================================================
    # Calculate distance to the closest orange cone
    if len(orange_cones) > 0:
        distances_to_orange = np.linalg.norm(orange_cones - vehicle_position, axis=1)
        min_dist_to_orange = np.min(distances_to_orange)
    else:
        min_dist_to_orange = float('inf')

    # Stop if we are close to an orange cone AND we've traveled far enough away from the start
    if min_dist_to_orange < DETECTION_RADIUS and distance_travelled > MIN_LAP_DISTANCE:
        print(f"Orange cones reached (Distance: {min_dist_to_orange:.2f}m) — Lap completed! Stopping simulation.")
        break

    step += 1


# ==========================================================
# 9. Save CSV trajectory
# ==========================================================
output_dir = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\output"
os.makedirs(output_dir, exist_ok=True)

csv_path = os.path.join(output_dir, "track_1.csv")

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["step", "x", "y"])
    writer.writerows(trajectory_log)

print(f" Trajectory saved to: {csv_path}")


# ==========================================================
# 10. Visualization (press 'q' to stop animation)
# ==========================================================
plt.figure(figsize=(12, 10))

stop_animation = False

def on_key(event):
    global stop_animation
    if event.key == "q":
        stop_animation = True
        plt.close()

plt.gcf().canvas.mpl_connect("key_press_event", on_key)

traj_array = np.array(trajectory_log)

for i, frame in enumerate(frames):

    if stop_animation:
        break

    plt.clf()

    # 1. Plot Cones (ADDED ORANGE CONES TO VISUALIZATION)
    plt.scatter(left_cones[:, 0], left_cones[:, 1], c="blue", s=20, zorder=5, label="Left Cones")
    plt.scatter(right_cones[:, 0], right_cones[:, 1], c="gold", s=20, zorder=5, label="Right Cones")
    if len(orange_cones) > 0:
        plt.scatter(orange_cones[:, 0], orange_cones[:, 1], c="darkorange", s=50, marker="^", zorder=6, label="Orange Cones")

    # 2. Plot ONLY the filtered, valid Delaunay edges
    valid_edges = frame.get("valid_edges", [])
    if valid_edges:
        lc = LineCollection(valid_edges, colors='gray', alpha=0.5, linewidths=1.5, zorder=1, label="Valid Delaunay Edges")
        plt.gca().add_collection(lc)

    # 3. Plot Delaunay Generated Waypoints
    waypoints = frame["waypoints"]
    if waypoints is not None and len(waypoints) > 0:
        plt.scatter(waypoints[:, 0], waypoints[:, 1], 
                    c='green', s=35, zorder=6, label="Waypoints")

    # 4. Plot Path Travelled (Historical)
    if i > 0:
        plt.plot(traj_array[:i, 1], traj_array[:i, 2], 'k--', linewidth=1.5, zorder=2, label="Travelled")

    # 5. Plot Planned Spline Path
    path = frame["path"]
    plt.plot(path[:, 1], path[:, 2], "r-", linewidth=3, zorder=4, label="Planned Spline")

    # 6. Plot Vehicle
    pos = frame["pos"]
    plt.scatter(pos[0], pos[1], c="black", s=150, zorder=7)

    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="upper right")
    plt.title(f"Delaunay Trackdrive Simulation | Step {i}")
    plt.pause(0.05)

plt.show()
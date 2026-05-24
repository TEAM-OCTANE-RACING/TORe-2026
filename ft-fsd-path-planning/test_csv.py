import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

from fsd_path_planning.full_pipeline.full_pipeline import PathPlanner
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes

# =====================================================
# VEHICLE LIMITS
# =====================================================
A_LAT_MAX =29.43           # m/s^2
V_VEHICLE_MAX = 47.0     # m/s

# =====================================================
# TRACK PARAMETERS
# =====================================================
TRACK_HALF_WIDTH = 1.5
RADIUS = 8.0
STRAIGHT_LEN = 12.0
CONE_SPACING = 2.5
NOISE_STD = 0.10

ADVANCE_DIST = 2.0       # meters per replanning step
MAX_STEPS = 100

np.random.seed(42)

# =====================================================
# 1. GENERATE HAIRPIN CENTERLINE
# =====================================================
s1 = np.arange(0, STRAIGHT_LEN, CONE_SPACING)
x1 = s1
y1 = np.zeros_like(s1)

theta = np.linspace(0, np.pi, int(np.pi * RADIUS / CONE_SPACING))
x2 = STRAIGHT_LEN + RADIUS * np.sin(theta)
y2 = RADIUS * (1 - np.cos(theta))

s3 = np.arange(0, STRAIGHT_LEN, CONE_SPACING)
x3 = STRAIGHT_LEN - s3
y3 = np.ones_like(s3) * (2 * RADIUS)

cx = np.concatenate([x1, x2, x3])
cy = np.concatenate([y1, y2, y3])
centerline = np.column_stack([cx, cy])

# =====================================================
# 2. COMPUTE NORMALS
# =====================================================
dx = np.gradient(cx)
dy = np.gradient(cy)
norm = np.sqrt(dx**2 + dy**2)

nx = -dy / norm
ny = dx / norm

# =====================================================
# 3. GENERATE SLAM CONES
# =====================================================
left_cones = np.column_stack([
    cx + TRACK_HALF_WIDTH * nx,
    cy + TRACK_HALF_WIDTH * ny
])

right_cones = np.column_stack([
    cx - TRACK_HALF_WIDTH * nx,
    cy - TRACK_HALF_WIDTH * ny
])

left_cones += np.random.normal(0, NOISE_STD, left_cones.shape)
right_cones += np.random.normal(0, NOISE_STD, right_cones.shape)

left_cones = left_cones[np.random.permutation(len(left_cones))]
right_cones = right_cones[np.random.permutation(len(right_cones))]

cones = [np.zeros((0, 2)) for _ in ConeTypes]
cones[ConeTypes.LEFT] = left_cones
cones[ConeTypes.RIGHT] = right_cones

# =====================================================
# 4. INITIAL VEHICLE STATE
# =====================================================
vehicle_position = np.array([-3.0, 0.0])
vehicle_direction = np.array([1.0, 0.0])

# =====================================================
# 5. PATH PLANNER
# =====================================================
planner = PathPlanner(MissionTypes.trackdrive)

all_center_points = []
all_curvatures = []
vehicle_trajectory = []

# =====================================================
# 6. RECEDING-HORIZON SIMULATION
# =====================================================
for step in range(MAX_STEPS):

    path = planner.calculate_path_in_global_frame(
        cones,
        vehicle_position,
        vehicle_direction
    )

    if path.shape[0] < 5:
        print("Path too short, stopping")
        break

    # Save full path (centerline)
    all_center_points.append(path[:, 1:3])
    all_curvatures.append(path[:, 3])

    # Advance vehicle along path
    s = path[:, 0]
    idx = np.searchsorted(s, ADVANCE_DIST)
    idx = min(idx, len(path) - 2)

    p_now = path[idx, 1:3]
    p_next = path[idx + 1, 1:3]

    vehicle_position = p_now
    vehicle_direction = (p_next - p_now)
    vehicle_direction /= np.linalg.norm(vehicle_direction)

    vehicle_trajectory.append(vehicle_position.copy())

    # Stop near end of track
    if np.linalg.norm(vehicle_position - centerline[-1]) < 2.0:
        print("Reached end of track")
        break

# =====================================================
# 7. CONCATENATE RESULTS
# =====================================================
center_points = np.vstack(all_center_points)
curvatures = np.hstack(all_curvatures)

# =====================================================
# 8. MAX VELOCITY FROM CURVATURE
# =====================================================
vmax = np.zeros_like(curvatures)

for i, k in enumerate(np.abs(curvatures)):
    if k < 1e-6:
        vmax[i] = V_VEHICLE_MAX
    else:
        vmax[i] = min(np.sqrt(A_LAT_MAX / k), V_VEHICLE_MAX)

# =====================================================
# 9. SAVE OUTPUT CSV
# =====================================================
output_dir = r"C:\Users\Ayush\ft-fsd-path-planning\OUTPUT"
os.makedirs(output_dir, exist_ok=True)

output_csv_path = os.path.join(output_dir, "planned_path_with_velocity.csv")

df_out = pd.DataFrame({
    "x_m": center_points[:, 0],
    "y_m": center_points[:, 1],
    "curvature_1pm": curvatures,
    "v_max_mps": vmax
})

df_out.to_csv(output_csv_path, index=False)

print(f"\n CSV saved successfully at:\n{output_csv_path}")

# =====================================================
# 10. VISUALIZATION
# =====================================================
plt.figure(figsize=(11, 7))

# Cones
plt.scatter(left_cones[:, 0], left_cones[:, 1],
            c="deepskyblue", s=50, label="Left cones")
plt.scatter(right_cones[:, 0], right_cones[:, 1],
            c="gold", s=50, label="Right cones")

# Path (velocity-colored)
sc = plt.scatter(
    center_points[:, 0],
    center_points[:, 1],
    c=vmax,
    cmap="viridis",
    s=15,
    label="Planned centerline"
)

# Vehicle trajectory
vehicle_traj = np.array(vehicle_trajectory)
plt.plot(vehicle_traj[:, 0], vehicle_traj[:, 1],
         "k--", linewidth=2, label="Vehicle trajectory")

plt.colorbar(sc, label="Max velocity (m/s)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("FSD Path Planning – Full Track with Velocity Profile")
plt.show()








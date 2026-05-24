import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

from fsd_path_planning.full_pipeline.full_pipeline import PathPlanner
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes

# =====================================================
# PARAMETERS
# =====================================================
LOOKAHEAD_DIST = 100.0      # meters (local planning horizon)
MOVE_DIST = 10            # meters vehicle moves along path (ONE step)

json_path = r"C:\Users\Ayush\layout-merchant\layouts\ecurie_track_1.json"

# =====================================================
# 1. LOAD JSON TRACK
# =====================================================
with open(json_path, "r") as f:
    track = json.load(f)

x = np.array(track["x"])
y = np.array(track["y"])
color = np.array(track["color"])

# Color mapping (as per layout description)
LEFT_COLOR = 1    # blue
RIGHT_COLOR = 2   # yellow

left_cones = np.column_stack([x[color == LEFT_COLOR], y[color == LEFT_COLOR]])
right_cones = np.column_stack([x[color == RIGHT_COLOR], y[color == RIGHT_COLOR]])

print("Left cones:", left_cones.shape)
print("Right cones:", right_cones.shape)

# =====================================================
# 2. COMPUTE TRACK MIDPOINTS (ROBUST CENTER ESTIMATE)
# =====================================================
def compute_midpoints(left, right):
    tree = KDTree(right)
    mids = []
    for lc in left:
        _, idx = tree.query(lc)
        mids.append(0.5 * (lc + right[idx]))
    return np.array(mids)

midpoints = compute_midpoints(left_cones, right_cones)

# =====================================================
# 3. INITIAL VEHICLE STATE (FROM TRACK GEOMETRY)
# =====================================================
vehicle_position = midpoints[0]

vehicle_direction = midpoints[5] - midpoints[0]
vehicle_direction /= np.linalg.norm(vehicle_direction)

# =====================================================
# 4. LEFT / RIGHT CONSISTENCY CHECK (IMPORTANT)
# =====================================================
lr_vec = right_cones.mean(axis=0) - left_cones.mean(axis=0)

# If cones are swapped, fix them
if np.cross(
    np.append(vehicle_direction, 0),
    np.append(lr_vec / np.linalg.norm(lr_vec), 0)
)[-1] < 0:
    left_cones, right_cones = right_cones, left_cones
    print("Swapped left/right cones to match heading")

# =====================================================
# 5. PACKAGE CONES
# =====================================================
cones = [np.zeros((0, 2)) for _ in ConeTypes]
cones[ConeTypes.LEFT] = left_cones
cones[ConeTypes.RIGHT] = right_cones

# =====================================================
# 6. PATH PLANNER
# =====================================================
planner = PathPlanner(MissionTypes.trackdrive)

# =====================================================
# 7. FIRST LOCAL PLAN
# =====================================================
path_1 = planner.calculate_path_in_global_frame(
    cones,
    vehicle_position,
    vehicle_direction
)

# Limit to lookahead distance
path_1 = path_1[path_1[:, 0] <= LOOKAHEAD_DIST]

print("First plan points:", path_1.shape[0])

# =====================================================
# 8. MOVE VEHICLE ONCE ALONG PATH
# =====================================================
s = path_1[:, 0]
idx = np.searchsorted(s, MOVE_DIST)
idx = min(idx, len(path_1) - 2)

new_position = path_1[idx, 1:3]
next_point = path_1[idx + 1, 1:3]

new_direction = next_point - new_position
new_direction /= np.linalg.norm(new_direction)

# =====================================================
# 9. SECOND LOCAL PLAN (AFTER MOTION)
# =====================================================
path_2 = planner.calculate_path_in_global_frame(
    cones,
    new_position,
    new_direction
)

path_2 = path_2[path_2[:, 0] <= LOOKAHEAD_DIST]

print("Second plan points:", path_2.shape[0])

# =====================================================
# 10. VISUALIZATION
# =====================================================
plt.figure(figsize=(10, 6))

# Cones
plt.scatter(left_cones[:, 0], left_cones[:, 1],
            c="deepskyblue", s=40, label="Left cones")
plt.scatter(right_cones[:, 0], right_cones[:, 1],
            c="gold", s=40, label="Right cones")

# First path
plt.plot(path_1[:, 1], path_1[:, 2],
         "r-", linewidth=3, label="Planned path (step 1)")

# Second path
plt.plot(path_2[:, 1], path_2[:, 2],
         "m--", linewidth=3, label="Planned path (step 2)")

# Vehicle positions
plt.scatter(vehicle_position[0], vehicle_position[1],
            c="black", s=120, marker=">", label="Start")

plt.scatter(new_position[0], new_position[1],
            c="green", s=120, marker=">", label="After move")

# Heading arrows
plt.quiver(vehicle_position[0], vehicle_position[1],
           vehicle_direction[0], vehicle_direction[1],
           scale=5, width=0.005, color="black")

plt.quiver(new_position[0], new_position[1],
           new_direction[0], new_direction[1],
           scale=5, width=0.005, color="green")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Local Path Planning → Move Once → Replan")
plt.show()

import json
import numpy as np
import matplotlib.pyplot as plt

from fsd_path_planning.full_pipeline.full_pipeline import PathPlanner
from fsd_path_planning.utils.cone_types import ConeTypes
from fsd_path_planning.utils.mission_types import MissionTypes

# ----------------------------
# Load JSON
# ----------------------------
with open("C:\\Users\\Ayush\\layout-merchant\\layouts\\fsg19.json", "r") as f:
    track = json.load(f)

x = np.array(track["x"])
y = np.array(track["y"])
color = np.array(track["color"])

# ----------------------------
# Extract cones
# ----------------------------
LEFT_COLOR = 2
RIGHT_COLOR = 1

left_cones = np.column_stack([x[color == LEFT_COLOR], y[color == LEFT_COLOR]])
right_cones = np.column_stack([x[color == RIGHT_COLOR], y[color == RIGHT_COLOR]])

cones = [np.zeros((0, 2)) for _ in ConeTypes]
cones[ConeTypes.LEFT] = left_cones
cones[ConeTypes.RIGHT] = right_cones

# ----------------------------
# Vehicle pose (FROM JSON)
# ----------------------------
vehicle_position = np.array(track["start_position"])

yaw = np.deg2rad(track["start_orientation"])
vehicle_direction = np.array([np.cos(yaw), np.sin(yaw)])

# ----------------------------
# Plan
# ----------------------------
planner = PathPlanner(MissionTypes.trackdrive)

path = planner.calculate_path_in_global_frame(
    cones,
    vehicle_position,
    vehicle_direction
)

# ----------------------------
# Plot
# ----------------------------
plt.figure(figsize=(10, 6))
plt.scatter(left_cones[:, 0], left_cones[:, 1], c="blue", label="Blue cones")
plt.scatter(right_cones[:, 0], right_cones[:, 1], c="yellow", label="Yellow cones")
plt.plot(path[:, 1], path[:, 2], "r-", linewidth=3, label="Planned path")
plt.scatter(vehicle_position[0], vehicle_position[1],
            c="black", s=120, marker=">", label="Start")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Correct JSON-based Path Planning")
plt.show()

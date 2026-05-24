import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

# Data
x = np.array([0, 1, 2, 3, 4])

y_top = np.array([2.1, 2.0, 2.0, 2.25, 2.27])
y_bottom = np.array([1.0, 0.95, 1.05, 1.02, 0.85])

# Create figure
fig, ax = plt.subplots(figsize=(10, 6))

# Background color
ax.set_facecolor("#e6e6e6")
fig.patch.set_facecolor("#e6e6e6")

# Top line (Blue)
ax.plot(x, y_top, marker='o', linewidth=3, markersize=10,
        label="Top Line")

# Bottom line (Yellow)
ax.plot(x, y_bottom, marker='o', linewidth=3, markersize=10,
        label="Bottom Line")

# Add numbers to each point (near markers)
for i in range(len(x)):
    ax.text(x[i]-0.08, y_top[i]-0.12, str(i), fontsize=16)
    ax.text(x[i]-0.08, y_bottom[i]-0.12, str(i), fontsize=16)

# Red arrow (triangle)
triangle = Polygon([[-0.5, 1.6], [-0.9, 1.45], [-0.9, 1.75]],
                   closed=True, color='red')
ax.add_patch(triangle)

# Formatting
ax.set_xlim(-1.2, 4.3)
ax.set_ylim(0.5, 2.8)

ax.set_xticks([])
ax.set_yticks([])

# Add legend
ax.legend(loc="upper right", fontsize=12)

# Thicker border
for spine in ax.spines.values():
    spine.set_linewidth(1.5)

plt.show()
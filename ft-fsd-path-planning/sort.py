import pandas as pd
import matplotlib.pyplot as plt

# Path to your sorted CSV
csv_path = r"C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\output\skidpad_centerline_3.csv"

# Load CSV
df = pd.read_csv(csv_path)

# Extract coordinates
x = df["x"].values
y = df["y"].values

# Plot
plt.figure(figsize=(8, 8))
plt.plot(x, y, 'b-', linewidth=2)
plt.axis("equal")
plt.grid(True)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Skidpad Centerline")
plt.show()

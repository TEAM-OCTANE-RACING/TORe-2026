import os
import glob
import time
import numpy as np
import matplotlib.pyplot as plt

# ==========================================================
# 1. VEHICLE DYNAMICS LIMITS
# ==========================================================
ay_max = 1.27 * 9.81    # ~12.46 m/s^2
ax_max = 0.64 * 9.81    # ~6.28 m/s^2
adec_max = 1.2 * 9.81   # ~11.77 m/s^2

# ==========================================================
# 2. LOAD LATEST TRAJECTORY DATA
# ==========================================================
# Look for the newest CSV in the current folder, or fallback to the output folder
search_patterns = ['*.csv', r'C:\Users\Ayush\ft-fsd-path-planning\fsd_path_planning\output\*optimal_lap2*.csv']
latest_file = None

for pattern in search_patterns:
    files = glob.glob(pattern)
    if files:
        latest_file = max(files, key=os.path.getctime)
        break

if not latest_file:
    raise FileNotFoundError("Could not find any optimal_lap2 CSV files!")

print(f"📊 Loading live telemetry data from: {os.path.basename(latest_file)}")

# Read CSV (Skipping the header row)
data = np.genfromtxt(latest_file, delimiter=',', skip_header=1)
x = data[:, 0]
y = data[:, 1]
heading = data[:, 2]
curvature = data[:, 3]
v = data[:, 4]

# ==========================================================
# 3. CALCULATE ACCELERATIONS & LAP TIME
# ==========================================================
N = len(v)
ay = np.zeros(N)
ax = np.zeros(N)

dx = np.diff(x)
dy = np.diff(y)
ds = np.sqrt(dx**2 + dy**2)

# Calculate elapsed time for each point to sync animation
v_avg = (v[:-1] + v[1:]) / 2.0
v_avg[v_avg < 0.5] = 0.5  # Safety against division by zero on standing start
dt = ds / v_avg

t_array = np.zeros(N)
t_array[1:] = np.cumsum(dt)
total_lap_time = t_array[-1]

for i in range(N - 1):
    ay[i] = (v[i]**2) * curvature[i]
    if ds[i] > 1e-5:
        ax[i] = (v[i+1]**2 - v[i]**2) / (2 * ds[i])

ay[-1] = (v[-1]**2) * curvature[-1]
ax[-1] = ax[-2]

print(f"⏱️ Calculated Lap Time: {total_lap_time:.2f} seconds")

# ==========================================================
# 4. GENERATE THE THEORETICAL FRICTION ELLIPSE
# ==========================================================
theta_accel = np.linspace(-np.pi/2, np.pi/2, 100)
ellipse_ax_accel = ax_max * np.cos(theta_accel)
ellipse_ay_accel = ay_max * np.sin(theta_accel)

theta_brake = np.linspace(np.pi/2, 3*np.pi/2, 100)
ellipse_ax_brake = adec_max * np.cos(theta_brake) 
ellipse_ay_brake = ay_max * np.sin(theta_brake)

ellipse_ax = np.concatenate((ellipse_ax_accel, ellipse_ax_brake))
ellipse_ay = np.concatenate((ellipse_ay_accel, ellipse_ay_brake))

# ==========================================================
# 5. LIVE ANIMATION SETUP
# ==========================================================
plt.ion()  # Turn on interactive mode for live plotting
fig, ax_plot = plt.subplots(figsize=(10, 8))
fig.canvas.manager.set_window_title('Live GG Diagram Telemetry')

# Plot the static boundaries
ax_plot.plot(ellipse_ay, ellipse_ax, 'orange', linewidth=3, label="Theoretical Tire Limit")
ax_plot.axhline(0, color='black', linewidth=0.5, linestyle='--')
ax_plot.axvline(0, color='black', linewidth=0.5, linestyle='--')

# Initialize empty scatter plot for the trail
scatter = ax_plot.scatter([], [], c=[], cmap='viridis', vmin=min(v), vmax=max(v), s=15, alpha=0.8, label="Trajectory Data points")
cbar = plt.colorbar(scatter)
cbar.set_label('Velocity (m/s)', rotation=270, labelpad=15)

# Initialize the "Current Position" red tracker dot
current_dot, = ax_plot.plot([], [], 'ro', markersize=10, markeredgecolor='white', markeredgewidth=1.5, zorder=5, label="Current State")

# Initialize live telemetry text box
telemetry_text = ax_plot.text(0.03, 0.97, '', transform=ax_plot.transAxes,
                              fontsize=12, verticalalignment='top', family='monospace',
                              bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))

# Formatting
ax_plot.set_title("Live Telemetry: Trajectory GG-Diagram", fontsize=14, fontweight='bold')
ax_plot.set_ylabel("Longitudinal Acceleration - $a_x$ ($m/s^2$)\n<-- Braking  |  Accelerating -->", fontsize=12)
ax_plot.set_xlabel("Lateral Acceleration - $a_y$ ($m/s^2$)\n<-- Right Turn  |  Left Turn -->", fontsize=12)
ax_plot.axis('equal')
ax_plot.grid(True, alpha=0.3)
ax_plot.legend(loc='upper right')
fig.tight_layout()

# ==========================================================
# 6. RUN THE ANIMATION LOOP (REAL-TIME SYNC)
# ==========================================================
print("🚀 Starting 1:1 Real-Time Playback...")

t_start = time.time()
last_idx = 0

while True:
    if not plt.fignum_exists(fig.number): # Stop if user closes the window
        break
        
    # Check actual wall-clock time passed since animation started
    t_elapsed = time.time() - t_start
    
    if t_elapsed >= total_lap_time:
        idx = N
    else:
        # Find the exact trajectory index that matches our current real-world time
        idx = np.searchsorted(t_array, t_elapsed)
    
    # Only update the plot if we've moved to a new point (saves CPU)
    if idx > last_idx:
        # Get data up to current frame
        current_ay = ay[:idx]
        current_ax = ax[:idx]
        current_v = v[:idx]
        
        # 1. Update the scatter trail
        scatter.set_offsets(np.column_stack((current_ay, current_ax)))
        scatter.set_array(current_v)
        
        # 2. Update the red tracker dot
        current_dot.set_data([ay[idx-1]], [ax[idx-1]])
        
        # 3. Update the live telemetry HUD
        hud_string = (f"LIVE TELEMETRY\n"
                      f"--------------\n"
                      f"Time:  {t_elapsed:>5.2f} / {total_lap_time:.2f} s\n"
                      f"Speed: {v[idx-1]:>5.1f} m/s\n"
                      f"Ax:    {ax[idx-1]:>5.2f} m/s²\n"
                      f"Ay:    {ay[idx-1]:>5.2f} m/s²")
        telemetry_text.set_text(hud_string)
        
        # Flush GUI events quickly without pausing artificially
        plt.pause(0.001)
        last_idx = idx

    if t_elapsed >= total_lap_time:
        break

print(f"🏁 Playback Finished. Simulated lap completed in {t_elapsed:.2f} seconds.")
plt.ioff() # Turn off interactive mode
plt.show() # Keep the window open at the end
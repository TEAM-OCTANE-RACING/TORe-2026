import numpy as np
from scipy.interpolate import splprep, splev

class TrajectoryOptimizer:
    """
    Unified library for Formula Student path optimization and velocity planning.
    Combines 1D Normal Shifting (Alpha-Parameterization) with a 3-Phase Friction 
    Ellipse Velocity Solver.
    """
    
    def __init__(self, vehicle_limits: dict):
        """
        Initialize the optimizer with the vehicle's dynamic limits.
        """
        self.ay_max = vehicle_limits.get('ay_max', 1.27 * 9.81)
        self.ax_max = vehicle_limits.get('ax_max', 0.64 * 9.81)
        self.adec_max = vehicle_limits.get('adec_max', 1.2 * 9.81)
        self.v_max_absolute = vehicle_limits.get('v_max', 21.427)

    def optimize_racing_line(self, centerline: np.ndarray, track_half_width: float, 
                             safety_margin: float = 0.1, iterations: int = 200) -> np.ndarray:
        """Optimizes geometry using Alpha-Parameterization and B-Splines."""
        if len(centerline) < 4:
            raise ValueError("Centerline must have at least 4 points for B-Spline.")

        x, y = centerline[:, 0].copy(), centerline[:, 1].copy()
        N = len(x)

        dx = np.gradient(x)
        dy = np.gradient(y)
        ds_grad = np.sqrt(dx**2 + dy**2) + 1e-6
        nx, ny = -dy / ds_grad, dx / ds_grad 

        alpha = np.zeros(N)
        bound_left = max(track_half_width - safety_margin, 0.0)
        bound_right = -max(track_half_width - safety_margin, 0.0)

        for _ in range(iterations):
            px, py = x + alpha * nx, y + alpha * ny
            
            px_avg, py_avg = np.copy(px), np.copy(py)
            px_avg[1:-1] = 0.5 * (px[:-2] + px[2:])
            py_avg[1:-1] = 0.5 * (py[:-2] + py[2:])

            move_x, move_y = px_avg - px, py_avg - py
            d_alpha = move_x * nx + move_y * ny
            
            alpha += d_alpha
            alpha = np.clip(alpha, bound_right, bound_left)

        opt_x, opt_y = x + alpha * nx, y + alpha * ny

        try:
            tck, _ = splprep([opt_x, opt_y], s=0.5)
            u_new = np.linspace(0, 1.0, N)
            x_smooth, y_smooth = splev(u_new, tck)
            
            dx_s, dy_s = splev(u_new, tck, der=1)
            ddx_s, ddy_s = splev(u_new, tck, der=2)
            
            heading = np.arctan2(dy_s, dx_s)
            curvature = (dx_s * ddy_s - dy_s * ddx_s) / ((dx_s**2 + dy_s**2)**1.5 + 1e-6)
        except Exception:
            x_smooth, y_smooth = opt_x, opt_y
            dx_s, dy_s = np.gradient(x_smooth), np.gradient(y_smooth)
            heading = np.arctan2(dy_s, dx_s)
            ddx_s, ddy_s = np.gradient(dx_s), np.gradient(dy_s)
            curvature = (dx_s * ddy_s - dy_s * ddx_s) / ((dx_s**2 + dy_s**2)**1.5 + 1e-6)

        s_array = np.zeros(N)
        s_array[1:] = np.cumsum(np.sqrt(np.diff(x_smooth)**2 + np.diff(y_smooth)**2))

        return np.column_stack((x_smooth, y_smooth, heading, curvature, s_array))

    def calculate_velocity_profile(self, s: np.ndarray, curvature: np.ndarray, standing_start: bool = True) -> np.ndarray:
        """Executes the 3-pass solver based on the GG-diagram."""
        N = len(s)
        ds = np.zeros(N)
        ds[:-1] = np.diff(s)
        ds[-1] = ds[-2] 
        
        # PHASE 1: Grip Limit
        v_max_curve = np.zeros(N)
        for i in range(N):
            k = abs(curvature[i])
            v_max_curve[i] = np.sqrt(self.ay_max / k) if k > 1e-6 else self.v_max_absolute
        v_max_curve = np.clip(v_max_curve, 0, self.v_max_absolute)

        # PHASE 2: Engine Limit
        v_fwd = np.copy(v_max_curve)
        if standing_start:
            v_fwd[0] = 0.0 
            
        for i in range(N - 1):
            ay_used = (v_fwd[i]**2) * abs(curvature[i])
            grip_ratio = min(1.0, ay_used / self.ay_max)
            ax_avail = self.ax_max * np.sqrt(1.0 - grip_ratio**2)
            v_reachable = np.sqrt(v_fwd[i]**2 + 2 * ax_avail * ds[i])
            v_fwd[i+1] = min(v_fwd[i+1], v_reachable)

        # PHASE 3: Braking Limit
        v_bwd = np.copy(v_fwd)
        for i in range(N - 2, -1, -1):
            ay_used = (v_bwd[i+1]**2) * abs(curvature[i+1])
            grip_ratio = min(1.0, ay_used / self.ay_max)
            adec_avail = self.adec_max * np.sqrt(1.0 - grip_ratio**2)
            v_brake_entry = np.sqrt(v_bwd[i+1]**2 + 2 * adec_avail * ds[i])
            v_bwd[i] = min(v_bwd[i], v_brake_entry)

        return v_bwd

    def calculate_lap_time(self, s: np.ndarray, v: np.ndarray) -> float:
        """Calculates theoretical lap time using trapezoidal integration."""
        ds = np.diff(s)
        v_avg = (v[:-1] + v[1:]) / 2.0
        v_avg[v_avg < 0.5] = 0.5 
        return np.sum(ds / v_avg)

    def generate_full_trajectory(self, centerline: np.ndarray, track_half_width: float, 
                                 safety_margin: float = 0.1, iterations: int = 200, 
                                 standing_start: bool = True) -> dict:
        """
        Master function: Runs Alpha-Param and Velocity Planning sequentially.
        Returns a dictionary with the full 5D profile and lap times.
        """
        geom_data = self.optimize_racing_line(centerline, track_half_width, safety_margin, iterations)
        x, y, heading, curvature, s_array = geom_data.T
        
        velocity = self.calculate_velocity_profile(s_array, curvature, standing_start)
        lap_time = self.calculate_lap_time(s_array, velocity)
        
        return {
            "trajectory_array": np.column_stack((x, y, heading, curvature, velocity)),
            "lap_time": lap_time
        }
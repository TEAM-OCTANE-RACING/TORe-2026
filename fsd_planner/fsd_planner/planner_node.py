import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from eufs_msgs.msg import ConeArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.spatial import Delaunay

class GlobalFSDPlanner(Node):
    def __init__(self):
        super().__init__('fsd_planner_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )
        
        self.cone_sub = self.create_subscription(ConeArray, '/planning/cones', self.cone_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/slam/odom', self.odom_callback, 1)

        self.path_pub = self.create_publisher(Path, '/target_path', 1)
        self.speed_pub = self.create_publisher(Float64MultiArray, '/target_speeds', 1)

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        self.mu = 0.8          
        self.gravity = 9.81    
        self.max_speed = 6.0   
        self.min_speed = 2.0   

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cone_callback(self, msg):
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)

        local_pts = []
        colors = [] # 0 for blue, 1 for yellow

        def process_cones(cone_list, color_code):
            for c in cone_list:
                # 1. Transform to Car's Local Frame
                dx = c.x - self.car_x
                dy = c.y - self.car_y
                lx = dx * cos_yaw + dy * sin_yaw
                ly = -dx * sin_yaw + dy * cos_yaw
                
                # 2. Strict Region of Interest (Only cones IN FRONT of the car)
                if 0.5 < lx < 25.0 and abs(ly) < 10.0:
                    local_pts.append([lx, ly])
                    colors.append(color_code)

        process_cones(msg.blue_cones, 0)
        process_cones(msg.yellow_cones, 1)

        if len(local_pts) < 4:
            return # Need at least 4 cones to form a meaningful track corridor

        points = np.array(local_pts)
        colors = np.array(colors)

        try:
            # 3. Compute Delaunay Triangulation
            tri = Delaunay(points)
            
            edges = set()
            for simplex in tri.simplices:
                edges.add(tuple(sorted([simplex[0], simplex[1]])))
                edges.add(tuple(sorted([simplex[1], simplex[2]])))
                edges.add(tuple(sorted([simplex[2], simplex[0]])))

            midpoints_local = []
            
            # 4. Extract valid Track "Rungs" (Blue-to-Yellow edges only)
            for i, j in edges:
                if colors[i] != colors[j]: # Must span across the track
                    p1, p2 = points[i], points[j]
                    dist = np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                    
                    if 1.5 < dist < 8.0: # Valid track width filter
                        midpoint = (p1 + p2) / 2.0
                        midpoints_local.append(midpoint)

            if len(midpoints_local) == 0:
                return

            midpoints_local = np.array(midpoints_local)

            # 5. Monotonic Progression Sort
            # Sort strictly by distance forward (Local X). 
            # This completely physically prevents U-Turns and Snaking!
            sort_idx = np.argsort(midpoints_local[:, 0])
            sorted_mps = midpoints_local[sort_idx]

            filtered_mps = []
            last_x = 0.0
            for mp in sorted_mps:
                if mp[0] > last_x + 0.5: # Must progress forward by at least 0.5m
                    filtered_mps.append(mp)
                    last_x = mp[0]

            if len(filtered_mps) < 2:
                return

            # 6. Transform Midpoints back to Global Map Frame
            global_m_x = [self.car_x]
            global_m_y = [self.car_y]

            for mp in filtered_mps:
                gx = self.car_x + mp[0] * cos_yaw - mp[1] * sin_yaw
                gy = self.car_y + mp[0] * sin_yaw + mp[1] * cos_yaw
                global_m_x.append(gx)
                global_m_y.append(gy)

            # 7. Generate Spline
            k_val = min(3, len(global_m_x) - 1)
            tck, u = splprep([global_m_x, global_m_y], s=1.0, k=k_val)
            
            u_new = np.linspace(0, 1, min(40, len(global_m_x) * 5))
            spline_x, spline_y = splev(u_new, tck)
            
            dx, dy = splev(u_new, tck, der=1)
            if k_val >= 2:
                ddx, ddy = splev(u_new, tck, der=2)
            else:
                ddx, ddy = np.zeros_like(dx), np.zeros_like(dy)

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            speed_msg = Float64MultiArray()

            for i in range(len(spline_x)):
                denom = (dx[i]**2 + dy[i]**2)**1.5
                curvature = abs(dx[i]*ddy[i] - dy[i]*ddx[i]) / denom if denom != 0 else 0.001
                radius = 1.0 / max(curvature, 0.001)
                
                target_v = float(np.clip(math.sqrt(self.mu * self.gravity * radius), self.min_speed, self.max_speed))

                pose = PoseStamped()
                pose.pose.position.x = float(spline_x[i]) 
                pose.pose.position.y = float(spline_y[i])
                pose.pose.position.z = 0.0 
                path_msg.poses.append(pose)
                speed_msg.data.append(target_v) 

            self.path_pub.publish(path_msg)
            self.speed_pub.publish(speed_msg)

        except Exception as e:
            self.get_logger().error(f"Delaunay Planner Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalFSDPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
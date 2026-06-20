from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. State Estimator
        Node(
            package='my_state_estimator',  
            executable='state_estimator_node',   
            name='state_estimator',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/imu/data', '/imu/data'), 
                ('/joint_states', '/eufs/joint_states'),
                ('/reset_simulation', '/reset_simulation'),
                ('/odometry/filtered', '/custom_odom') 
            ]
        ),

        # 2. EKF-SLAM
        Node(
            package='my_slam_pkg',
            executable='eufs_slam_node',
            name='eufs_slam_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/perception/cones', '/ground_truth/cones'), 
                ('/odometry/filtered', '/custom_odom'),
                ('/reset_simulation', '/reset_simulation')
            ]
        ),

        # 3. Path Planner
        Node(
            package='fsd_planner',
            executable='planner_node',
            name='path_planner',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 4. Hybrid Controller
        Node(
            package='my_controller_pkg',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/target_path', '/target_path'),
                ('/cmd', '/cmd'),
                ('/reset_simulation', '/reset_simulation')
            ]
        ),
        
        # 5. Mission Manager
        Node(
            package='my_controller_pkg',
            executable='mission_manager',
            name='mission_manager',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        
    ])

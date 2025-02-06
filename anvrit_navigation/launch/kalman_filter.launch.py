from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'odom0': '/gazebo/robot/odometry',
                'imu0': '/gazebo/robot/imu/data'
            }]
        )
    ])

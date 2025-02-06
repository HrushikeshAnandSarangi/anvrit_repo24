import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    gps_wpf_dir = get_package_share_directory("anvrit_description")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_world.world")

    # Process the URDF file
    urdf = os.path.join(gps_wpf_dir, 'urdf', 'anvrit.urdf.xacro')
    robot_description = xacro.process_file(urdf).toxml()

    # Set up model paths
    models_dir = os.path.join(gps_wpf_dir, "models")
    ros_distro = os.getenv('ROS_DISTRO', 'humble')
    ros_models_dir = os.path.join(f"/opt/ros/{ros_distro}/share/sonoma_raceway/models")
    models_dir = os.pathsep.join([models_dir, ros_models_dir])

    # Configure GAZEBO_MODEL_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + models_dir
    else:
        gazebo_model_path = models_dir
    set_gazebo_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)

    # Set TurtleBot3 model environment variable
    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both'
    )

    # Start Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory("anvrit_navigation")
    anvrit_dir = get_package_share_directory("anvrit_description")
    anvrit_launch = os.path.join(anvrit_dir, "launch")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    
    # Launch configuration variables specific to simulation
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='True',
        description='Whether to start mapviz')

    # Specify the actions
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(anvrit_launch, 'sonoma_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navstat.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items()
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'autostart': 'True',
            }.items()),
    ])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)

    return ld
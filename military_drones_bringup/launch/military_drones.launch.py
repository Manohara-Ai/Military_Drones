import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('military_drones_bringup')
    pkg_gazebo = get_package_share_directory('military_drones_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Whether to launch RViz"
    )

    # Launch Ignition Gazebo with your world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gazebo,
            'worlds',
            'world.sdf'
        ])}.items(),
    )

    # Bridge ROS <-> Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'military_drones_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Rviz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_bringup, 'config', 'X3.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    flight_controller_1 = Node(
        package='military_drones_control',
        executable='flight_controller',
        parameters=[{
            'namespace': 'X3_1',
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_z': 0.0
        }],
        output='screen',
    )

    flight_controller_2 = Node(
        package='military_drones_control',
        executable='flight_controller',
        parameters=[{
            'namespace': 'X3_2',
            'initial_x': 5.0,
            'initial_y': 0.0,
            'initial_z': 0.0
        }],
        output='screen',
    )

    flight_controller_3 = Node(
        package='military_drones_control',
        executable='flight_controller',
        parameters=[{
            'namespace': 'X3_3',
            'initial_x': 0.0,
            'initial_y': 5.0,
            'initial_z': 0.0
        }],
        output='screen',
    )

    drone_gui = Node(
        package='military_drones_control',
        executable='drone_gui',
        output='screen',
    )

    object_recognizer = Node(
        package='military_drones_control',
        executable='object_recognizer',
        output='screen',
    )

    return LaunchDescription([
        declare_rviz,
        gz_sim,
        bridge,
        # rviz,
        object_recognizer,
        flight_controller_1,
        flight_controller_2,
        flight_controller_3,
        drone_gui
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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

    # Conditionally set NVIDIA GPU variables if the driver exists
    env_actions = []
    if os.path.exists("/proc/driver/nvidia/version"):
        env_actions.extend([
            SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1"),
            SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia"),
            SetEnvironmentVariable(name="__VK_LAYER_NV_optimus", value="NVIDIA_only"),
        ])

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

    # Flight controllers
    flight_controllers = []
    positions = [(0.0,0.0,0.0), (5.0,0.0,0.0), (0.0,5.0,0.0)]
    namespaces = ['X3_1','X3_2','X3_3']
    for ns, pos in zip(namespaces, positions):
        flight_controllers.append(
            Node(
                package='military_drones_control',
                executable='flight_controller',
                parameters=[{'namespace': ns,
                             'initial_x': pos[0],
                             'initial_y': pos[1],
                             'initial_z': pos[2]}],
                output='screen'
            )
        )

    drone_gui = Node(
        package='military_drones_control',
        executable='drone_gui',
        output='screen',
    )

    return LaunchDescription(
        env_actions +
        [declare_rviz, gz_sim, bridge] +
        flight_controllers +
        [drone_gui]
    )

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter # Added SetParameter

def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('military_drones_bringup')
    pkg_project_gazebo = get_package_share_directory('military_drones_gazebo')
    pkg_project_description = get_package_share_directory('military_drones_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the URDF file
    urdf_file = os.path.join(pkg_project_description, 'models', 'quadrotor', 'quadrotor.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg_project_gazebo, 'worlds', 'surveillance_world.sdf']), ' -r']
        }.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}] # use_sim_time handled by SetParameter below
    )

    # 3. Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'quadrotor', '-topic', 'robot_description', '-z', '0.4'],
    )

    # 4. RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'surveillance_world.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 5. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'military_drones_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        # This global parameter ensures every node in this launch file uses the /clock topic
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        rviz,
    ])
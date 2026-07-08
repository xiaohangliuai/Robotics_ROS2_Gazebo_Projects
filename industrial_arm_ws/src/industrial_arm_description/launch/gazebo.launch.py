import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # FORCE ENVIRONMENT VARIABLES BEFORE LAUNCHING NODES
    # This ensures Gazebo Sim completely sees the ROS 2 Jazzy plugin paths
    if 'GZ_SIM_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] += ':/opt/ros/jazzy/lib'
    else:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '/opt/ros/jazzy/lib'

    pkg_dir = get_package_share_directory('industrial_arm_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'industrial_arm', '-z', '0.1'],
        output='screen'
    )

    # BRIDGE GAZEBO CLOCK TO ROS 2 /clock TOPIC
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", os.path.join(pkg_dir, 'config', 'arm_controllers.yaml')],
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_sim,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])
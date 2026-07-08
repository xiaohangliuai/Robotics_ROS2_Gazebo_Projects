import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("industrial_arm", package_name="industrial_arm_moveit_config")
        .robot_description(file_path="config/industrial_arm.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_planning_scene=True)
        .to_moveit_configs()
    )

    # MoveIt2 move_group node tied to simulation time
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    # RViz2 visualizer with MoveIt plugin
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        run_move_group_node,
        rviz_node
    ])

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    panda_description = get_package_share_directory("panda_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                panda_description, "urdf", "panda_urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty_world.sdf")
    conveyor_speed_arg = DeclareLaunchArgument(
        name="conveyor_speed",
        default_value="0.15",
        description="Conveyor surface speed in metres per second",
    )

    world_path = PathJoinSubstitution([
            panda_description,
            "worlds",
            LaunchConfiguration("world_name")
        ]
    )

    share_root = str(Path(panda_description).parent.resolve())
    gazebo_model_path = os.path.join(panda_description, 'models')
    resource_paths = os.pathsep.join([share_root, panda_description, gazebo_model_path])

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        resource_paths
        )
    gazebo_model_path_env = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        resource_paths
        )

    ros_distro = os.environ["ROS_DISTRO"]
    # Humble uses the old Ignition plugin name; Iron and newer use gz_ros2_control.
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " mesh_prefix:=file://" + panda_description + "/meshes"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-x", "0.0",  
            "-y", "0.0",  
            "-z", "0.0",  
            "-R", "0.0", 
            "-P", "0.0",
            "-Y", "0.0", # Yaw (in radians, e.g., 1.57 for 90 degrees)
        ],
    )


    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    start_conveyor = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gz", "topic",
                    "-t", "/model/conveyor/link/belt_link/track_cmd_vel",
                    "-m", "gz.msgs.Double",
                    "-p", ["data: ", LaunchConfiguration("conveyor_speed")],
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        conveyor_speed_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gazebo_model_path_env,
        gz_ros2_bridge,
        ros_gz_image_bridge,
        start_conveyor,
    ])

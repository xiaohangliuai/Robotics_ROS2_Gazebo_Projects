import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    panda_description_dir = get_package_share_directory("panda_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        panda_description_dir, "urdf", "panda_urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "jazzy" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    gui_env = {"QT_QPA_PLATFORM": "xcb"}
    if os.environ.get("DISPLAY"):
        gui_env["DISPLAY"] = os.environ["DISPLAY"]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    launch_nodes = [model_arg, robot_state_publisher_node]

    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        joint_state_publisher_gui_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            additional_env=gui_env,
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(panda_description_dir, "rviz", "display.rviz")],
            additional_env=gui_env,
        )
        launch_nodes.extend([joint_state_publisher_gui_node, rviz_node])
    else:
        launch_nodes.append(
            Node(
                package="launch_ros",
                executable="echo",
                arguments=["No graphical display detected; skipping RViz and joint_state_publisher_gui."],
            )
        )

    return LaunchDescription(launch_nodes)

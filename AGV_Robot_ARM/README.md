# AGV Robot Arm

This repository contains a ROS 2 + Gazebo project for simulating an AGV robot arm with a camera sensor, mobile base, and arm joints.

## Project Overview

The project is organized into two main ROS 2 packages:

- `my_robot_description`: URDF/Xacro robot description, meshes, and RViz configuration
- `my_robot_bringup`: launch files, Gazebo bridge configuration, and world assets

## Features

- Mobile robot base simulation
- Robot arm joint control
- Gazebo world launch
- Camera sensor publishing image and camera info topics
- ROS 2 bridge for Gazebo communication

## Requirements

Before building and running the project, make sure you have:

- Ubuntu with ROS 2 installed
- `colcon` build tools
- `xacro`
- Gazebo and ROS-Gazebo bridge packages (`ros_gz_sim`, `ros_gz_bridge`)

## Build

From the repository root:

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you want to build only the relevant packages:

```bash
colcon build --symlink-install --packages-select my_robot_description my_robot_bringup
```

## Run the Simulation

Launch the Gazebo world and robot:

```bash
ros2 launch my_robot_bringup my_robot_gazebo.xml
```

This launch file starts:

- `robot_state_publisher`
- Gazebo simulation
- the Gazebo-ROS bridge

## View the Camera Feed

The camera sensor publishes image data to:

```bash
/camera/image_raw
```

You can view it with:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/camera/image_raw` in the GUI.

## Repository Structure

```text
AGV_Robot_ARM/
├── my_robot_bringup/
│   ├── launch/
│   ├── config/
│   └── worlds/
├── my_robot_description/
│   ├── urdf/
│   ├── launch/
│   └── rviz/
└── README.md
```

## Notes

- The robot description is defined using Xacro files under `my_robot_description/urdf/`.
- Gazebo communication is handled through the bridge configuration in `my_robot_bringup/config/gazebo_bridge.yaml`.
- If you encounter issues with missing ROS 2 packages, install the required `ros_gz` packages for your ROS 2 distribution.

## Troubleshooting

If the simulation does not start correctly:

1. Make sure the workspace is sourced.
2. Verify that all required ROS 2 and Gazebo packages are installed.
3. Check the terminal output for missing package or plugin errors.
4. Rebuild the workspace after changing URDF/Xacro files.

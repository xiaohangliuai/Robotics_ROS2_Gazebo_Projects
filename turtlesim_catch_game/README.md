# Turtlesim Catch Them All

A simple ROS 2 project that turns Turtlesim into a small catch game. A controller turtle tracks and catches spawned turtles, while the spawner adds new targets over time.

## Demo

![Turtlesim Catch Game demo](gif/Turtlesim_game.gif)

## What this project does

- Spawns new turtles at random positions with a configurable frequency.
- Tracks the currently alive turtles and publishes that list to a topic.
- Moves the main turtle toward a target turtle based on its current pose.
- Calls a service to remove a caught turtle from the simulation.
- Supports choosing whether to catch the closest turtle first or simply take the first one in the list.

## Project structure

- `my_robot_bringup/` - launch files and configuration
- `my_robot_interfaces/` - custom ROS interfaces
- `turtlesim_catch_them_all/` - main game logic and nodes
- `gif/` - demo GIF asset

## Requirements

- ROS 2 installed and sourced
- Python 3
- `colcon` build tool

## Build

From the project root:

```bash
source /opt/ros/<your_ros2_distro>/setup.bash
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch my_robot_bringup turtle_launch.xml
```

This launch file starts:

- `turtlesim_node`
- `turtle_spawner`
- `turtle_controller`

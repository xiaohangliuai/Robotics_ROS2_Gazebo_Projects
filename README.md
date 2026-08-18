# Robotics Projects

This workspace contains a few robotics projects that reflect my learning journey in ROS 2, Gazebo, navigation, localization, and control:

## [Panda Pick and Place](panda_pick_and_place/)

A ROS 2 Jazzy and Gazebo simulation where a Franka Emika Panda robot uses
OpenCV color detection and MoveIt 2 motion planning to identify, pick up, and
sort red, green, and blue objects into their matching containers.

- Detects colored objects from a simulated RGB camera feed.
- Supports picking one selected color or sorting all three automatically.
- Uses MoveIt 2 for arm motion planning and ROS 2 controllers for the arm and
  gripper.

![Panda color-detection pick-and-place demo](panda_pick_and_place/src/gif/panda_color_sorting_demo-ezgif.com-video-to-gif-converter.gif)

[View setup, launch, and usage instructions](panda_pick_and_place/README.md).

## AGV_Robot_ARM
An AGV robot arm project with camera focused on robot description, bringup, and simulation for a mobile robotic arm system.

![AGV Robot Arm](AGV_Robot_ARM/gif/agv_robot_arm.gif)

## turtlesim_catch_game
A playful ROS 2 simulation project built around the turtlesim environment, where the goal is to catch and interact with turtles in a simple game-style setup.

![Turtlesim Catch Game](turtlesim_catch_game/gif/Turtlesim_game.gif)

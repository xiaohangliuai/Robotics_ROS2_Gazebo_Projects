# Turtlesim Catch Them All

A simple ROS 2 demonstration package that turns the classic turtlesim environment into a small "catch them all" game.

## Overview

This package spawns random turtles into the turtlesim window and controls a main turtle to chase and catch them. It uses ROS 2 nodes, services, and custom messages/interfaces to coordinate the game flow.

## What the project does

- Spawns new turtles at random positions with a configurable frequency.
- Tracks the currently alive turtles and publishes that list to a topic.
- Moves the main turtle toward a target turtle based on its current pose.
- Calls a service to remove a caught turtle from the simulation.
- Supports choosing whether to catch the closest turtle first or simply take the first one in the list.

## Main nodes

### turtle_spawner

Responsible for:
- spawning turtles through the turtlesim spawn service
- keeping track of alive turtles
- publishing the current turtle list
- handling the catch service request by killing the target turtle

### turtle_controller

Responsible for:
- subscribing to the main turtle pose
- subscribing to the alive turtle list
- choosing the target turtle to chase
- publishing velocity commands to move the main turtle toward the target
- calling the catch service when the target is reached

## Custom interfaces

This package depends on custom messages and services from the my_robot_interfaces package:
- Turtle
- TurtleArray
- CatchTurtle

## Dependencies

- ROS 2
- Python 3
- rclpy
- turtlesim
- geometry_msgs
- my_robot_interfaces

## How to run

1. Build the workspace:

   ```bash
   cd ~/ros2_study_ws
   colcon build
   source install/setup.bash
   ```

2. Start turtlesim:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

3. Start the spawner node:

   ```bash
   ros2 run turtlesim_catch_them_all turtle_spawner
   ```

4. Start the controller node:

   ```bash
   ros2 run turtlesim_catch_them_all turtle_controller
   ```

## Notes

You can adjust the behavior of the spawner and controller using ROS 2 parameters, for example:
- turtle_name_prefix
- spawn_frequency
- catch_closest_turtle_first

## Project purpose

This package is a beginner-friendly ROS 2 example that demonstrates:
- node communication
- service calls
- topic publishing/subscription
- parameter usage
- custom interfaces in a small interactive simulation

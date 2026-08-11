# Panda Color-Detection Pick and Place

A ROS 2 simulation in which a Franka Emika Panda robot detects colored objects
with an RGB camera and uses MoveIt 2 to pick a selected object and move it to a
drop location.

The project brings together Gazebo, `ros2_control`, MoveIt 2, OpenCV, and TF2.
The vision node publishes detected object coordinates in the robot base frame,
and the pick-and-place node plans and executes the arm and gripper motions.

## Demo

<video controls width="900">
  <source src="./src/gif/panda_color_detection_pick%26place.mp4" type="video/mp4">
  Your browser does not support embedded MP4 video.
</video>

[Watch or download the MP4 demo](./src/gif/panda_color_detection_pick%26place.mp4)

## Packages

| Package | Purpose |
| --- | --- |
| `panda_description` | Panda URDF/Xacro, meshes, camera, Gazebo models, and world |
| `panda_controller` | Arm and gripper `ros2_control` configuration |
| `panda_moveit` | MoveIt 2 planning configuration and RViz setup |
| `panda_vision` | OpenCV color detection and camera-to-base TF conversion |
| `panda_bringup` | Combined simulation launch file |
| `pymoveit2` | Python MoveIt 2 interface and the pick-and-place node |

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Sim and the ROS-Gazebo integration packages
- MoveIt 2
- `ros2_control` and its controller packages
- OpenCV, `cv_bridge`, NumPy, and TF transformations for Python
- `colcon` and `rosdep`

Using a desktop installation of ROS 2 and installing the remaining package
dependencies with `rosdep` is recommended.

## Build

Clone the repository as a ROS 2 workspace, then run:

```bash
cd panda_pick_and_place
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Rebuild and source the workspace again after changing Python, launch, URDF, or
configuration files.

## Run the simulation

Start Gazebo, the robot controllers, MoveIt/RViz, and the color detector:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

Once the simulation and controllers are ready, open a second terminal, source
the workspace, and select the object color to pick:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B
```

Valid color values are:

- `R` — red
- `G` — green
- `B` — blue

The node waits for the first matching detection on `/color_coordinates`, locks
its position, and performs the pick-and-place sequence once.

## How it works

1. The simulated camera publishes images on `/camera/image_raw`.
2. `panda_vision` segments red, green, and blue objects in HSV color space.
3. The detected image position is transformed from `camera_link` to
   `panda_link0` and published on `/color_coordinates`.
4. The pick-and-place node accepts the selected color, plans arm motions with
   MoveIt 2, and commands the gripper trajectory controller directly.
5. The robot approaches the object, grips it, moves to the drop pose, releases
   it, and returns to its start configuration.


## Troubleshooting

- If a package cannot be found, source both ROS 2 and `install/setup.bash` in
  every terminal.
- If the robot does not move, wait until the arm and gripper controllers are
  active before starting the pick-and-place node.
- If no target is detected, confirm that `/camera/image_raw` is publishing and
  that `/color_coordinates` contains the selected color.
- The color detector opens an OpenCV window, so it needs access to a graphical
  display.

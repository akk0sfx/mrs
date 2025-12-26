# diff_robot_sim

ROS 2 (Jazzy) diff-drive robot simulation for Gazebo Sim with LiDAR + SLAM.

## Launch

```bash
ros2 launch diff_robot_sim full_pipeline.launch.py \
  use_gui:=true use_rviz:=true \
  world:=$HOME/Documents/GitHub/mrs/src/diff_robot_sim/worlds/main.world
```

## SLAM-only demo flow

1. Launch:

   ```bash
   ros2 launch diff_robot_sim full_pipeline.launch.py use_gui:=true use_rviz:=true
   ```

2. Drive the robot:
   - Teleop is disabled by default. Start with:

     ```bash
     ros2 launch diff_robot_sim full_pipeline.launch.py use_gui:=true use_rviz:=true enable_teleop:=true
     ```

   - Use WASD in the launch terminal (Space = stop, X = exit).
   - If input does not work, run it manually in another terminal:

     ```bash
     ros2 run diff_robot_sim wasd_teleop.py
     ```

3. View the map:
   - RViz uses `Fixed Frame: map` and shows `/scan` and `/map`.
   - The map updates once the robot moves.

## Quick checks

```bash
ros2 topic echo /scan
ros2 topic list | rg '^/map$|/map_updates'
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

## macOS GUI notes

- The launch files set `QT_MAC_WANTS_LAYER=1` for Gazebo GUI and RViz to avoid the
  "Dock icon without window" issue on macOS.
- If windows still do not appear, verify you are running in a local GUI session
  (not SSH) and that your shell does not override `QT_*` variables.

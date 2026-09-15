# Obstacle-avoidance waypoint simulation

Barebones **ROS 2 Jazzy** + **Gazebo Harmonic** simulation of a box-shaped
differential-drive vehicle that **drives itself** through a randomized sequence
of XY waypoints. Walls, a hill, and a ramp sit between the goals; the vehicle
plans around them. There is no keyboard teleop.

## What you get

| Piece | Role |
| --- | --- |
| `worlds/obstacle_course.sdf` | Gazebo world: ground, perimeter, interior walls, hill, ramp, vehicle |
| `waypoint_generator` | Samples free-space XY goals and publishes `/waypoints` |
| `fake_lidar` | Raycasts the known walls and publishes `/scan` |
| `navigator` | Reads pose + next goal + scan, publishes `/cmd_vel` on its own |
| `sim.launch.py` | Starts Gazebo, the ROS–Gazebo bridge, and the three nodes |

The vehicle is a blue box on two wheels. Modeling detail is intentionally low.

## Requirements

- Ubuntu 24.04 (Noble)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Gazebo Harmonic via `ros-jazzy-ros-gz` (vendor packages)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz \
  python3-colcon-common-extensions \
  python3-pytest
```

## Build and run

From the repository root (this is a colcon workspace):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select obstacle_avoidance
source install/setup.bash

# Gazebo GUI + RViz + autonomous navigator
ros2 launch obstacle_avoidance sim.launch.py

# Headless (CI / no display)
ros2 launch obstacle_avoidance sim.launch.py gui:=false rviz:=false

# Reproducible waypoint set
ros2 launch obstacle_avoidance sim.launch.py seed:=7 num_waypoints:=5
```

The vehicle starts at the origin. Watch it visit the green cylinders
(`WP1` …) in order. RViz shows odometry, the planned path, lidar rays, and
the current goal.

### Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Navigator → Gazebo DiffDrive |
| `/odom` | `nav_msgs/Odometry` | Gazebo → navigator |
| `/scan` | `sensor_msgs/LaserScan` | Fake lidar (replaceable) |
| `/waypoints` | `geometry_msgs/PoseArray` | Latched mission |
| `/planned_path` | `nav_msgs/Path` | A* result |
| `/current_goal` | `geometry_msgs/PoseStamped` | Active waypoint |
| `/nav_status` | `std_msgs/String` | Human-readable state |

## Where to plug in your own logic

Comments in the source mark the same three extension points:

1. **Waypoint generation** — `obstacle_avoidance/mission.py`, function
   `sample_waypoints` (the ROS node only publishes the result). Swap the
   rejection sampler for a mission file, a coverage pattern, or a ROS service
   that accepts goals from a UI.

2. **Global planner** — `obstacle_avoidance/planner.py`, function `plan_path`.
   The default is 8-connected A* on a grid inflated from the wall rectangles.
   Drop in Nav2, RRT, D* Lite, etc. Return a list of `(x, y)` poses.

3. **Local controller / sensing** — `obstacle_avoidance/navigator.py`, method
   `Navigator.compute_cmd_vel`, and `obstacle_avoidance/fake_lidar.py`.
   `compute_cmd_vel` already receives current pose, the next goal, the planned
   path, and a `LaserScan`. To use a real Gazebo lidar, add a `gpu_lidar`
   plugin to the vehicle SDF, bridge it to `/scan`, and stop launching
   `fake_lidar`.

World geometry lives in `obstacle_avoidance/obstacle_map.py` (`OBSTACLES`).
After editing it, regenerate the SDF so Gazebo and the planner stay in sync:

```bash
python3 -c "from obstacle_avoidance.world_sdf import write_world; print(write_world())"
```

## Tests

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select obstacle_avoidance --event-handlers console_direct+
```

The unit tests check free space, A* going around walls, and that the committed
world file matches the generator. They do not start Gazebo.

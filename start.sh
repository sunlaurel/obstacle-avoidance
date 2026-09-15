source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select obstacle_avoidance
source install/setup.bash
ros2 launch obstacle_avoidance sim.launch.py

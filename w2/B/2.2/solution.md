# Running the sim

First clone into `~/ros_ws/src` the `volksbot_driver` repo with its dependency `epos2_motor_controller` and then the `hsfd_gazebo_simulation` repo.

Then run:
```bash
cd ~/ros_ws/
colcon build
source install/setup.bash
ros2 launch hsfd_gazebo_simulation simulation.launch.py robot_pkg_name:=volksbot_driver robot_urdf_file:=volksbot.urdf.xacro world_name:=hsfd_campus_example
```
which should launch the simulation.

Control the bot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

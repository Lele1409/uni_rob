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

Get the msg definitions:
```bash
ros interface show .../.../...
```

Build and run the recorder:
```bash
cd ~/ros_ws/src/
rm -rf ./trajectory_recorder
cp -r ~/uni_rob/w2/B/2.2/trajectory_recorder ./trajectory_recorder
cd ..
colcon build
ros run trajectory_recorder recorder
```

Control the bot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once finished moving, close the trajectory_recorder.
Then run:
```bash
gnuplot
plot 'odometry.txt' u 1:2
```


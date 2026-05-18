# Running the sim

```bash
cd <repo-root>
```

Make sure the submodules in `./ros_ws/src` are loaded. See the `README.md`.

Then run to build:
```bash
cd ./ros2_ws/
colcon build
source install/setup.bash
```

Run to launch the simulation:
```bash
ros launch hsfd_gazebo_simulation simulation.launch.py robot_pkg_name:=volksbot_driver robot_urdf_file:=volksbot.urdf.xacro world_name:=hsfd_campus_example
```

Getting the message definition is possible via:
```bash
ros interface show .../.../...
```

Run the recorder:
```bash
ros run trajectory_recorder recorder
```

Control the bot:
```bash
ros run teleop_twist_keyboard teleop_twist_keyboard
```

Once finished moving, close the trajectory_recorder.
Then run, in the same directory where the recorder was run:
```bash
gnuplot
plot 'odometry.txt' u 1:2
```


To setup the environment for this task, run:
```bash
cd ~/uni_rob/
colcon build
source install/bash.setup
```

To start the recording, run:
```bash
cd w3/A/3.2
rm imu.txt odometry.txt
ros2 run trajectory_recorder recorder
```

In a second terminal, play the bag which recorded the imu and odometry topics by running:
```bash
ros2 bag play ~/uni_rob/w3/random_bags/a7
```

Quit the process in the first terminal, then visualize the data by running:
```bash
gnuplot
plot 'odometry.txt' u 1:2
plot 'imu.txt' u 1:2
```
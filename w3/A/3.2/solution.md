cd ~/uni_rob/
colcon build
source install/bash.setup
cd w3/A/3.2
rm imu.txt odometry.txt
ros2 run trajectory_recorder recorder

ros2 bag play ~/uni_rob/w3/A/3.1/rosbag2_2026_05_26-19_31_47/rosbag2_2026_05_26-19_31_47_0.mcap

quit first terminal

gnuplot
plot 'odometry.txt' u 1:2

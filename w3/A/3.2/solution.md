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

## Änderungen am recorder

Das originale `recorder.cpp` schrieb Rohbeschleunigungen direkt in `imu.txt` ohne Positionsberechnung. Die neue Version integriert die IMU-Daten zu einer Trajektorie: Geschwindigkeitskomponenten werden via `v_i = a_i * Δt` berechnet, der Betrag `v = sqrt(vx²+vy²+vz²)` ergibt die Schrittweite, und der Drehwinkel wird aus `angular_velocity.z * Δt` akkumuliert. Die berechneten x/y-Koordinaten werden in `imu.txt` gespeichert.

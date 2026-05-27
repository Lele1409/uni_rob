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

## Änderungen am recorder

Das originale `recorder.cpp` schrieb Rohbeschleunigungen direkt in `imu.txt` ohne Positionsberechnung. Die neue Version integriert die IMU-Daten zu einer Trajektorie: Geschwindigkeitskomponenten werden via `v_i = a_i * Δt` berechnet, der Betrag `v = sqrt(vx²+vy²+vz²)` ergibt die Schrittweite, und der Drehwinkel wird aus `angular_velocity.z * Δt` akkumuliert. Die berechneten x/y-Koordinaten werden in `imu.txt` gespeichert.

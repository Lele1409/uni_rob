# HowTo

## Terminal 1: Roboter verbinden und Driver starten

````bash
ssh robot@IP
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch volksbot_driver volksbot.py
````

## Terminal 2: Teleop starten

````bash
ssh robot@IP
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
````

## Terminal 3: Topic prüfen und Bagfile aufnehmen

````bash
ssh robot@IP
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic list
ros2 bag record /odom 
````

### Nach der Fahrt Aufnahme mit Ctrl + C beenden und prüfen

````bash
ros2 bag info Dateiname
````

### Lokales Terminal : Bagfile kopieren

````bash
scp -r robot@10.33.130.107:~/ros2_ws/meine_runde ~/#
````

### VM: Bagfile abspielen

````bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play ~/meine_runde --loop
````

## Änderungen am recorder

Das originale `recorder.cpp` subscribte auf `/odom` und `/imu` und schrieb Rohdaten in die Ausgabedateien. Die neue Version subscribt auf die korrekten Topics `odom` und `imu/data_raw` und nutzt `const nav_msgs::msg::Odometry&` statt `SharedPtr` als Callback-Parameter.

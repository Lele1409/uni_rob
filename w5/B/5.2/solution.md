# B5.2 – EKF / Robot Localization

## Setup

Install the packages:

```
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-teleop-twist-keyboard
```

(nicht `ros2-robot-localization`, das existiert nicht)

Build and source the workspace, then launch in three separate terminals:

**Terminal 1** – Gazebo-Simulation (veröffentlicht `/odom`, `/imu/data`, `/clock`):
```
ros2 launch hsfd_gazebo_simulation simulation.launch.py robot_pkg_name:=volksbot_driver robot_urdf_file:=volksbot.urdf.xacro
```

**Terminal 2** – EKF-Node:
```
colcon build --packages-select launch_robot_local
source install/setup.bash
ros2 launch launch_robot_local launch.this.py use_sim_time:=true
```

**Terminal 3** – Tastatursteuerung:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Hinweis: `use_sim_time:=true` ist notwendig, da Gazebo eine eigene Simulationsuhr (`/clock`) veröffentlicht. Ohne diesen Parameter wartet der EKF-Node auf eine Echtzeituhr und verarbeitet keine Daten.

---

## Abweichungen von den linearen KF-Annahmen beim Volksbot

Der lineare Kalman-Filter setzt vier Bedingungen voraus. Beim Volksbot sind alle vier in der Praxis verletzt:

**1. Alle Sensoren sind perfekt modelliert**
Die Radencoder des Volksbots sind anfällig für Radschlupf, insbesondere bei scharfen Kurven oder auf unebenem Untergrund. Das IMU hat temperaturbedingten Drift und einen konstanten Bias, der sich über die Zeit verändert. Beide Sensoren können nur näherungsweise modelliert werden.

**2. Das System kann komplett durch lineare Zustandsübergänge beschrieben werden**
Der Volksbot ist ein Differentialantrieb mit nicht-linearer Kinematik: Eine Änderung der Ausrichtung (yaw) beeinflusst die x- und y-Komponenten der Geschwindigkeit trigonometrisch. Das ist inhärent nicht-linear. Der EKF approximiert dies durch Linearisierung mittels Jacobi-Matrizen im aktuellen Arbeitspunkt – daher der "Extended" Kalman Filter.

**3. Das Sensorrauschen ist normalverteilt**
Radschlupf erzeugt nicht-Gauß'sches Rauschen mit schweren Schwänzen (seltene, aber große Ausreißer). Bodenunebenheiten und Oberflächenreibungsunterschiede führen zu systematischen, nicht zufälligen Fehlern. Das IMU-Rauschen ist zwar annähernd Gauß'sch, hat aber durch Vibrationen des Roboters zusätzliche strukturierte Störanteile.

**4. Der geschätzte Zustand ist normalverteilt und unimodal**
In symmetrischen Umgebungen kann der wahre Zustand des Roboters multimodal sein (mehrere gleich wahrscheinliche Positionen). Außerdem kann bei längerem Betrieb ohne Korrektur durch absolute Positionsmessungen die Positionsungewissheit so groß werden, dass sie nicht mehr durch eine einzelne Gauß-Kurve beschreibbar ist.

---

## Konfiguration (ekf.yaml)

### Topic-Mapping

| Parameter | Topic | Bedeutung |
|-----------|-------|-----------|
| `odom0` | `odom` | Odometrie des Volksbots (Position + Geschwindigkeit) |
| `imu0` | `imu/data` | IMU-Daten (Orientierung, Winkelgeschwindigkeit, Linearbeschleunigung) |

### Sensor-Konfigurationsvektoren

Format: `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`

**odom0_config** – verwendet werden Position (x, y, z) und Translationsgeschwindigkeiten (vx, vy, vz) sowie Gierrate (vyaw). Orientierung kommt vom IMU, daher hier `false`.

**imu0_config** – verwendet werden Orientierung (roll, pitch, yaw), Winkelgeschwindigkeiten und Linearbeschleunigungen (ax, ay, az). Der Hinweis in der Aufgabe, Linearbeschleunigungen für Translation zu ignorieren, wurde bewusst nicht übernommen, da der Volksbot auf ebenem Boden fährt und die Beschleunigungsdaten des IMU stabile Informationen liefern. Bei unebenem Terrain wäre es sinnvoller, ax und ay auf `false` zu setzen, um Vibrationsstörungen auszuschließen.

`imu0_relative: true` ist gesetzt, da das IMU keine absolute Ausrichtung kennt – die initiale Orientierung wird als Referenz verwendet.

`imu0_remove_gravitational_acceleration: true` ist nötig, weil das IMU des Volksbots die Erdbeschleunigung nicht intern herausrechnet.

### cmd_vel als Prior

Die Aufgabe nennt `cmd_vel` als Prior. In `robot_localization` entspricht das dem Parameter `use_control`. Dieser ist auf `false` gesetzt, weil die Odometrie bereits die tatsächlich gemessenen Radgeschwindigkeiten liefert – sie ist informativer als die kommandierten Geschwindigkeiten (`cmd_vel`), da sie Schlupf und mechanische Verzögerungen automatisch widerspiegelt. `cmd_vel` als Control-Input macht hauptsächlich Sinn, wenn keine Odometrie verfügbar ist oder wenn die Odometrie stark verrauscht ist.

Falls `use_control: true` gewünscht ist, müssen zusätzlich `acceleration_limits` und `control_config` korrekt auf die Kinematik des Volksbots abgestimmt werden (bereits im yaml vorbereitet).

### two_d_mode

`two_d_mode: true` ist gesetzt, da der Volksbot auf ebenem Boden fährt und 3D-Zustandsanteile (z, roll, pitch) ignoriert werden können. Das reduziert Rechenaufwand und verhindert, dass kleine Bodenunebenheiten die Schätzung destabilisieren.

---

## Parameter-Experimente

### process_noise_covariance

Die Diagonalwerte der Prozessrauschkovarianz bestimmen, wie stark der Filter nach jedem Prädiktionsschritt seiner eigenen Schätzung vertraut. Höhere Werte → Filter vertraut Messungen mehr, reagiert schneller auf Änderungen, aber Output ist rauschiger. Kleinere Werte → glattere Schätzung, aber träge Reaktion auf plötzliche Bewegungsänderungen.

Beobachtung: Erhöhen der Werte für `vx` und `vyaw` (Positionen 6 und 11 in der Matrix) verbessert die Reaktionszeit bei Kurvenfahrten, führt aber zu mehr Rauschen in der geraden Fahrt.

### Rejection Thresholds

`odom0_pose_rejection_threshold: 5.0` und `odom0_twist_rejection_threshold: 1.0` filtern Ausreißer in Odometrie-Messungen (Mahalanobis-Distanz). Kleinere Werte machen den Filter robuster gegen Schlupf-Spikes, können aber dazu führen, dass valide Messungen nach einer Pause (z.B. Neustart) abgelehnt werden.

`imu0_pose_rejection_threshold: 0.8` ist vergleichsweise streng und filtert größere IMU-Ausreißer zuverlässig heraus.

### initial_estimate_covariance

Mit sehr kleinen Werten (`1e-9`) wird der Filter sehr sicher über den Startzustand initialisiert. Das führt dazu, dass die ersten Messungen kaum Einfluss haben. Bei einem unbekannten Startzustand wären größere Werte (z.B. `1.0`) sinnvoller, damit der Filter schnell konvergiert.

---

## Trajektorienvergleich

*[Hier Screenshots aus RViz2 einfügen: gefilterte Trajektorie (EKF-Output auf `/odometry/filtered`) vs. rohe Odometrie (auf `/wheel/odometry`) vs. IMU-integrierte Trajektorie]*

Erwartetes Ergebnis:
- Rohe Odometrie: driftet bei Kurvenfahrten, da Radschlupf nicht korrigiert wird
- IMU allein: integriert Winkelgeschwindigkeiten, akkumuliert Drift über Zeit
- EKF-Fusion: glattere, konsistentere Schätzung durch gegenseitige Korrektur beider Sensoren

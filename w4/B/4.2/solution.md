# Lösung Aufgabe B4.2 - Online-Linienerkennung

Implementiert in `ros2_ws/src/online_line_finder/src/online_line_finder.cpp`.

Genutzte Folien: Online-Linienkonstruktion (Folie 202), Online-Linienfinder (Folie 203).

## Algorithmus

Sei `a_j, ..., a_k` die aktuell konstruierte Linie. Ein neuer Punkt `a_{k+1}` wird nur dann zur Linie hinzugefügt, wenn alle drei Bedingungen erfüllt sind:

1. **Abstands-Prüfung** (keine Linien durchs Nichts):  
   `||a_k, a_{k+1}||` ≤ `max_point_dist`  und  `||a_{k-1}, a_{k+1}||` ≤ `max_point_dist`

2. **Lokale Prüfung** (Luftlinien-Argument lokal):  
   `||a_{k-1}, a_{k+1}|| / (||a_{k-1}, a_k|| + ||a_k, a_{k+1}||)` ≥ `1 - ε`

3. **Globale Prüfung** (Luftlinien-Argument über die ganze Linie):  
   `||a_j, a_{k+1}|| / Σ||a_i, a_{i+1}||` ≥ `1 - ε(k)`

Schlägt eine Bedingung fehl, wird die aktuelle Linie geschlossen und `a_{k+1}` beginnt eine neue. Eine Linie gilt als gefunden, wenn sie ≥ `min_points` Punkte enthält.

Die kartesische Umrechnung der Polar-Scandaten erfolgt über:  
`x = r · cos(angle_min + i · angle_increment)`,  `y = r · sin(angle_min + i · angle_increment)`

## Parametrisierung

| Parameter | Typ | Standard | Beschreibung |
|---|---|---|---|
| `epsilon` | double | 0.98 | ε für lokale Prüfung |
| `epsilon_k` | double | 0.98 | ε(k) für globale Prüfung (hier konstant) |
| `min_points` | int | 5 | Mindestpunkte für eine Linie |
| `max_point_dist` | double | 0.2 | Max. Abstand zwischen Punkten (m) |
| `scan_topic` | string | `/scanout/scan` | Zu abonnierendes LaserScan-Topic |

Für den Husarion-Roboter (feinere Winkelauflösung) ggf. `max_point_dist` auf ~0.5 m reduzieren.

## Bauen

```bash
cd <repo-root>/ros2_ws
colcon build --packages-select online_line_finder
source install/setup.bash
```

## Ausführen

### 1. Bag abspielen

```bash
ros2 bag play w4/B/4.2/lidar_bag/ --loop
```

### 2. Node starten

Direkt mit Parametern:

```bash
ros2 run online_line_finder online_line_finder --ros-args -p epsilon:=0.02 -p epsilon_k:=0.02 -p min_points:=5 -p max_point_dist:=0.03
```

Oder über das Launch-File:

```bash
ros2 launch online_line_finder line_finder.launch.py epsilon:=0.02 epsilon_k:=0.02 min_points:=5 max_point_dist:=0.03
```

Der Node gibt pro Scan die Anzahl erkannter Linien aus:

```
[online_line_finder] Scan processed: 361 input points → 5 lines
```

## RViz einrichten

```bash
rviz2
```

Fixed Frame auf den Frame-ID der Scans setzen (z.B. `os_sensor`), dann folgende Displays hinzufügen:

| Display | Topic / Setting |
|---|---|
| LaserScan | `/scanout/scan` — zeigt die rohen Scan-Punkte |
| Marker | `/lines` — zeigt die erkannten Linien (grün) |

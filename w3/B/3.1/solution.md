# Lösung Aufgabe B3.1 — Scanmatching (ICP)

Implementiert in `ros2_ws/src/basic_icp/src/BasicICP.cpp`.

Genutzte Folien: Korrespondenzen (Folie 193), Lu/Milios Lemma (Folie 182), ICP-Algorithmus (Folie 195).

## Bauen

```bash
cd <repo-root>/ros2_ws
colcon build --packages-select basic_icp
source install/setup.bash
```

## RViz starten

Dies geht über den Befehl: `rviz2`

Zuerst muss wie in "RViz einrichten" eine Konfiguration erstellt werden.

Diese kann dann nun neu geladen werden.

## Ausführen

### 1. Bag abspielen

```bash
ros2 bag play w3/B/3.1/icp_bag/ --loop
```

### 2. Initiale Transformation anschauen (optional)

Vor dem ICP-Node kann man die initiale Lage der beiden Scans mit dem Static Transform Publisher prüfen:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 model laser
```

**Diesen Prozess beenden, bevor der ICP-Node gestartet wird** — sonst gibt es Konflikte zwischen dem statischen Publisher und den vom ICP-Node gesendeten Transformationen.

### 3. ICP-Node starten

```bash
ros2 run basic_icp basic_icp
```

Der Node iteriert mit **0,25 Hz** und gibt pro Iteration Transformation und RMSE-Fehler aus:

```
[basic_icp_node] Iteration 0 | tx=0.1234  ty=-0.0567  theta=3.8200 deg | RMSE=0.0423
```

## RViz einrichten

```bash
rviz2
```

Fixed Frame auf `model` setzen, dann folgende Displays hinzufügen:

| Display | Topic / Setting |
|---|---|
| LaserScan | `/model` — zeigt die Modell-Punktwolke |
| LaserScan | `/scan` — zeigt den Datenscan |
| Marker | `/icp_correspondences` — Linien zwischen Punktpaaren |
| TF | — zeigt die iterativ aktualisierte `laser`-Frame-Position |

Diese Konfiguration dann in `w3/B/3.1/rvz_config.rviz` speichern.
